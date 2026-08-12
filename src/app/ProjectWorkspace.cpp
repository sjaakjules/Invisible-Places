#include "app/ProjectWorkspace.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string_view>

namespace invisible_places::app::workspace {
namespace {

constexpr std::string_view kDataToken = "@data";
constexpr std::string_view kWorkspaceToken = "@workspace";
constexpr std::string_view kRenderToken = "@local-renders";
constexpr std::uint64_t kFnvOffsetBasis = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

std::string TrimAscii(std::string value) {
    const auto isSpace = [](unsigned char character) {
        return std::isspace(character) != 0;
    };
    value.erase(
        value.begin(),
        std::find_if(value.begin(), value.end(), [&](char character) {
            return !isSpace(static_cast<unsigned char>(character));
        }));
    value.erase(
        std::find_if(value.rbegin(), value.rend(), [&](char character) {
            return !isSpace(static_cast<unsigned char>(character));
        }).base(),
        value.end());
    return value;
}

std::string LowercaseAscii(std::string value) {
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

bool WritePathMarker(
    const std::filesystem::path& markerPath,
    const std::filesystem::path& selectedRoot,
    std::string_view emptyMessage,
    std::string* errorMessage) {
    if (selectedRoot.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = std::string{emptyMessage};
        }
        return false;
    }
    std::error_code createError;
    std::filesystem::create_directories(selectedRoot, createError);
    if (createError) {
        if (errorMessage != nullptr) {
            *errorMessage = "Could not create " + selectedRoot.string() +
                            ": " + createError.message();
        }
        return false;
    }

    const auto temporaryPath = markerPath.string() + ".tmp";
    std::ofstream marker{temporaryPath, std::ios::trunc};
    if (!marker.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Could not write " + markerPath.string() + ".";
        }
        return false;
    }
    marker << selectedRoot.lexically_normal().string() << '\n';
    marker.flush();
    const bool wrote = marker.good();
    marker.close();
    if (!wrote || marker.fail()) {
        std::error_code cleanupError;
        std::filesystem::remove(temporaryPath, cleanupError);
        if (errorMessage != nullptr) {
            *errorMessage = "Could not finish " + markerPath.string() + ".";
        }
        return false;
    }
    std::error_code replaceError;
    std::filesystem::rename(temporaryPath, markerPath, replaceError);
    if (replaceError) {
        std::error_code removeError;
        std::filesystem::remove(markerPath, removeError);
        replaceError.clear();
        std::filesystem::rename(temporaryPath, markerPath, replaceError);
    }
    if (replaceError) {
        if (errorMessage != nullptr) {
            *errorMessage = "Could not activate " + markerPath.string() +
                            ": " + replaceError.message();
        }
        return false;
    }
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

std::string RecoveryTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
#if defined(_WIN32)
    localtime_s(&local, &seconds);
#else
    localtime_r(&seconds, &local);
#endif
    std::ostringstream stream;
    stream << std::put_time(&local, "%Y%m%d-%H%M%S");
    return stream.str();
}

std::string SanitizeRecoveryStem(std::string value) {
    for (auto& character : value) {
        const auto code = static_cast<unsigned char>(character);
        if (!std::isalnum(code) && character != '-' && character != '_') {
            character = '_';
        }
    }
    return value.empty() ? std::string{"document"} : value;
}

std::vector<std::filesystem::path> Components(
    const std::filesystem::path& path) {
    std::vector<std::filesystem::path> components;
    for (const auto& component : path) {
        components.push_back(component);
    }
    return components;
}

std::filesystem::path JoinComponents(
    const std::vector<std::filesystem::path>& components,
    std::size_t first) {
    std::filesystem::path result;
    for (std::size_t index = first; index < components.size(); ++index) {
        result /= components[index];
    }
    return result;
}

std::optional<std::size_t> FindComponent(
    const std::vector<std::filesystem::path>& components,
    std::string_view name,
    std::size_t first = 0U) {
    const auto wanted = LowercaseAscii(std::string{name});
    for (std::size_t index = first; index < components.size(); ++index) {
        if (LowercaseAscii(components[index].string()) == wanted) {
            return index;
        }
    }
    return std::nullopt;
}

bool HasToken(
    const std::filesystem::path& path,
    std::string_view token) {
    if (path.empty() || path.is_absolute()) {
        return false;
    }
    const auto components = Components(path);
    return !components.empty() && components.front().generic_string() == token;
}

std::filesystem::path RemoveFirstComponent(
    const std::filesystem::path& path) {
    const auto components = Components(path);
    return components.size() <= 1U
               ? std::filesystem::path{}
               : JoinComponents(components, 1U);
}

bool LexicallyWithin(
    const std::filesystem::path& path,
    const std::filesystem::path& root,
    std::filesystem::path* relative) {
    if (path.empty() || root.empty()) {
        return false;
    }
    const auto normalizedPath = path.lexically_normal();
    const auto normalizedRoot = root.lexically_normal();
    auto candidate = normalizedPath.lexically_relative(normalizedRoot);
    if (candidate.empty() && normalizedPath != normalizedRoot) {
        return false;
    }
    const auto components = Components(candidate);
    if (!components.empty() && components.front() == "..") {
        return false;
    }
    if (relative != nullptr) {
        *relative = candidate;
    }
    return true;
}

std::filesystem::path PortableUnderRoot(
    const std::filesystem::path& path,
    const std::filesystem::path& root,
    std::string_view token) {
    if (path.empty() || HasToken(path, token)) {
        return path;
    }
    std::filesystem::path relative;
    if (!LexicallyWithin(path, root, &relative)) {
        return path;
    }
    return (std::filesystem::path{token} / relative).lexically_normal();
}

std::filesystem::path ResolveToken(
    const std::filesystem::path& path,
    const std::filesystem::path& root,
    std::string_view token) {
    if (!HasToken(path, token)) {
        return path;
    }
    const auto remainder = RemoveFirstComponent(path);
    return remainder.empty()
               ? root.lexically_normal()
               : (root / remainder).lexically_normal();
}

bool SceneAssociationToken(const std::filesystem::path& path) {
    const auto value = path.generic_string();
    return value.starts_with("__scene_group__/");
}

template <typename Transform>
void TransformAssociations(
    std::vector<std::filesystem::path>* paths,
    Transform transform) {
    if (paths == nullptr) {
        return;
    }
    for (auto& path : *paths) {
        if (!path.empty() && !SceneAssociationToken(path)) {
            path = transform(path);
        }
    }
}

template <typename Transform>
void TransformWaterEffectLayers(
    std::vector<invisible_places::water::WaterEffectLayer>* layers,
    Transform transform) {
    if (layers == nullptr) {
        return;
    }
    for (auto& layer : *layers) {
        layer.targetLayerSourcePath = transform(layer.targetLayerSourcePath);
    }
}

template <typename Transform>
void TransformWaterRuntimeCaches(
    std::vector<invisible_places::serialization::WaterRippleRuntimeCacheDocument>* caches,
    Transform transform) {
    if (caches == nullptr) {
        return;
    }
    for (auto& cache : *caches) {
        cache.supportLayerPath = transform(cache.supportLayerPath);
    }
}

template <typename Transform>
void TransformWaterPathCache(
    std::optional<invisible_places::water::WaterPathCache>* cache,
    Transform transform) {
    if (cache != nullptr && cache->has_value()) {
        cache->value().supportLayerPath =
            transform(cache->value().supportLayerPath);
    }
}

template <typename Transform>
void TransformWaterSceneState(
    invisible_places::serialization::WaterSceneStateDocument* state,
    Transform transform) {
    if (state == nullptr) {
        return;
    }
    TransformWaterEffectLayers(&state->rippleLayers, transform);
    TransformWaterEffectLayers(&state->fieldLayers, transform);
    TransformWaterPathCache(&state->pathCache, transform);
    TransformWaterRuntimeCaches(&state->rippleRuntimeCaches, transform);
    state->dynamicMeshPath = transform(state->dynamicMeshPath);
}

template <typename Transform>
void TransformWaterSources(
    invisible_places::serialization::WaterSourcesDocument* document,
    Transform transform) {
    if (document == nullptr) {
        return;
    }
    TransformWaterEffectLayers(&document->rippleLayers, transform);
    TransformWaterEffectLayers(&document->fieldLayers, transform);
    TransformWaterPathCache(&document->pathCache, transform);
    TransformWaterRuntimeCaches(&document->rippleRuntimeCaches, transform);
    document->dynamicMeshFlowSettings.meshPath =
        transform(document->dynamicMeshFlowSettings.meshPath);
}

template <typename DataTransform, typename WorkspaceTransform, typename RenderTransform>
void TransformProject(
    invisible_places::serialization::ProjectDocument* document,
    DataTransform dataTransform,
    WorkspaceTransform workspaceTransform,
    RenderTransform renderTransform) {
    if (document == nullptr) {
        return;
    }

    document->selectedLayerPath = dataTransform(document->selectedLayerPath);
    for (auto& layer : document->layers) {
        layer.sourcePath = dataTransform(layer.sourcePath);
        layer.selectedSceneVariantPath =
            dataTransform(layer.selectedSceneVariantPath);
    }
    for (auto& group : document->scenePointCloudGroups) {
        for (auto& role : group.roleSources) {
            role.analysisSourcePath = dataTransform(role.analysisSourcePath);
            role.displaySourcePath = dataTransform(role.displaySourcePath);
        }
    }
    for (auto& shot : document->cameraShots) {
        TransformAssociations(&shot.associatedLayerPaths, dataTransform);
    }

    document->activeAnimationPath =
        workspaceTransform(document->activeAnimationPath);
    document->lastAnimationPath =
        workspaceTransform(document->lastAnimationPath);
    for (auto& animation : document->savedAnimations) {
        animation.filePath = workspaceTransform(animation.filePath);
        TransformAssociations(
            &animation.associatedLayerPaths,
            dataTransform);
    }

    document->renderJobSettings.outputDirectory =
        renderTransform(document->renderJobSettings.outputDirectory).string();
    for (auto& preset : document->exportPresets) {
        preset.settings.outputDirectory =
            renderTransform(preset.settings.outputDirectory).string();
    }
    if (document->tempExportPreset.has_value()) {
        document->tempExportPreset->settings.outputDirectory =
            renderTransform(
                document->tempExportPreset->settings.outputDirectory).string();
    }

    TransformWaterEffectLayers(&document->waterRippleLayers, dataTransform);
    TransformWaterEffectLayers(&document->waterFieldLayers, dataTransform);
    TransformWaterPathCache(&document->waterPathCache, dataTransform);
    TransformWaterRuntimeCaches(
        &document->waterRippleRuntimeCaches,
        dataTransform);
    document->waterDynamicMeshFlowSettings.meshPath =
        dataTransform(document->waterDynamicMeshFlowSettings.meshPath);
    for (auto& state : document->waterSceneStates) {
        TransformWaterSceneState(&state, dataTransform);
    }
}

}  // namespace

std::filesystem::path LocalSavedDirectory(
    const std::filesystem::path& dataRoot) {
    if (dataRoot.filename() == "ExhibitionScene") {
        return dataRoot.parent_path().parent_path() / "Saved";
    }
    return dataRoot.parent_path() / "Saved";
}

std::filesystem::path WorkspaceMarkerPath(
    const std::filesystem::path& dataRoot) {
    return LocalSavedDirectory(dataRoot).parent_path() /
           kWorkspaceMarkerFilename;
}

std::filesystem::path SharedDataMarkerPath(
    const std::filesystem::path& localDataRoot) {
    return LocalSavedDirectory(localDataRoot).parent_path() /
           kSharedDataMarkerFilename;
}

std::filesystem::path ResolveSharedDataDirectory(
    const std::filesystem::path& localDataRoot,
    const std::filesystem::path& requestedDataRoot) {
    if (const char* value = std::getenv(kSharedDataEnvironmentVariable);
        value != nullptr) {
        const auto trimmed = TrimAscii(value);
        if (!trimmed.empty()) {
            return std::filesystem::path{trimmed}.lexically_normal();
        }
    }
    std::ifstream marker{SharedDataMarkerPath(localDataRoot)};
    std::string configured;
    if (marker.is_open() && std::getline(marker, configured)) {
        configured = TrimAscii(std::move(configured));
        if (!configured.empty()) {
            return std::filesystem::path{configured}.lexically_normal();
        }
    }
    return (requestedDataRoot.empty() ? localDataRoot : requestedDataRoot)
        .lexically_normal();
}

bool WorkspaceEnvironmentOverrideActive() {
    const char* value = std::getenv(kWorkspaceEnvironmentVariable);
    return value != nullptr && !TrimAscii(value).empty();
}

bool SharedDataEnvironmentOverrideActive() {
    const char* value = std::getenv(kSharedDataEnvironmentVariable);
    return value != nullptr && !TrimAscii(value).empty();
}

std::filesystem::path ResolveAuthoredWorkspaceDirectory(
    const std::filesystem::path& dataRoot) {
    if (const char* value = std::getenv(kWorkspaceEnvironmentVariable);
        value != nullptr) {
        const auto trimmed = TrimAscii(value);
        if (!trimmed.empty()) {
            return std::filesystem::path{trimmed}.lexically_normal();
        }
    }

    std::ifstream marker{WorkspaceMarkerPath(dataRoot)};
    std::string configured;
    if (marker.is_open() && std::getline(marker, configured)) {
        configured = TrimAscii(std::move(configured));
        if (!configured.empty()) {
            return std::filesystem::path{configured}.lexically_normal();
        }
    }
    return LocalSavedDirectory(dataRoot).lexically_normal();
}

bool WriteWorkspaceMarker(
    const std::filesystem::path& dataRoot,
    const std::filesystem::path& authoredRoot,
    std::string* errorMessage) {
    return WritePathMarker(
        WorkspaceMarkerPath(dataRoot),
        authoredRoot,
        "Choose an authored workspace folder.",
        errorMessage);
}

bool WriteSharedDataMarker(
    const std::filesystem::path& localDataRoot,
    const std::filesystem::path& sharedDataRoot,
    std::string* errorMessage) {
    return WritePathMarker(
        SharedDataMarkerPath(localDataRoot),
        sharedDataRoot,
        "Choose a shared source-data folder.",
        errorMessage);
}

std::optional<FileRevision> ReadFileRevision(
    const std::filesystem::path& path,
    std::string* errorMessage) {
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    if (path.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = "The document path is empty.";
        }
        return std::nullopt;
    }
    std::error_code metadataError;
    const bool exists = std::filesystem::exists(path, metadataError);
    if (metadataError) {
        if (errorMessage != nullptr) {
            *errorMessage = "Could not inspect " + path.string() + ": " +
                            metadataError.message();
        }
        return std::nullopt;
    }
    if (!exists) {
        return FileRevision{};
    }
    if (!std::filesystem::is_regular_file(path, metadataError) ||
        metadataError) {
        if (errorMessage != nullptr) {
            *errorMessage = "The save target is not a regular file: " +
                            path.string();
        }
        return std::nullopt;
    }

    std::ifstream input{path, std::ios::binary};
    if (!input.is_open()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Could not read " + path.string() +
                            " to verify its cloud revision.";
        }
        return std::nullopt;
    }
    std::uint64_t hash = kFnvOffsetBasis;
    std::uint64_t size = 0U;
    std::array<char, 64U * 1024U> buffer{};
    while (input) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const auto count = input.gcount();
        if (count < 0) {
            break;
        }
        for (std::streamsize index = 0; index < count; ++index) {
            hash ^= static_cast<unsigned char>(buffer[static_cast<std::size_t>(index)]);
            hash *= kFnvPrime;
        }
        size += static_cast<std::uint64_t>(count);
    }
    if (input.bad()) {
        if (errorMessage != nullptr) {
            *errorMessage = "Could not finish reading " + path.string() +
                            " to verify its cloud revision.";
        }
        return std::nullopt;
    }
    return FileRevision{
        .exists = true,
        .sizeBytes = size,
        .contentHash = hash,
    };
}

bool FileRevisionMatches(
    const std::filesystem::path& path,
    const FileRevision& expected,
    std::string* errorMessage) {
    const auto current = ReadFileRevision(path, errorMessage);
    return current.has_value() && current.value() == expected;
}

std::filesystem::path ConflictRecoveryDirectory(
    const std::filesystem::path& localSavedRoot) {
    return (localSavedRoot / "conflict_recovery").lexically_normal();
}

bool PreserveConflictRecoveryCopy(
    const std::filesystem::path& stagedPath,
    const std::filesystem::path& intendedTarget,
    const std::filesystem::path& localSavedRoot,
    std::filesystem::path* recoveryPath,
    std::string* errorMessage) {
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    const auto directory = ConflictRecoveryDirectory(localSavedRoot) /
                           RecoveryTimestamp();
    std::error_code createError;
    std::filesystem::create_directories(directory, createError);
    if (createError) {
        if (errorMessage != nullptr) {
            *errorMessage = "Could not create local conflict recovery: " +
                            createError.message();
        }
        return false;
    }
    const auto filename = intendedTarget.filename().string();
    const auto stem = SanitizeRecoveryStem(
        filename.empty() ? std::string{"document.json"} : filename);
    auto output = directory / stem;
    for (std::uint32_t suffix = 2U;
         std::filesystem::exists(output) && suffix < 10000U;
         ++suffix) {
        output = directory /
                 (SanitizeRecoveryStem(intendedTarget.stem().string()) + "_" +
                  std::to_string(suffix) + intendedTarget.extension().string());
    }
    std::error_code copyError;
    std::filesystem::copy_file(
        stagedPath,
        output,
        std::filesystem::copy_options::overwrite_existing,
        copyError);
    if (copyError) {
        if (errorMessage != nullptr) {
            *errorMessage = "Could not preserve the local conflict copy: " +
                            copyError.message();
        }
        return false;
    }
    if (recoveryPath != nullptr) {
        *recoveryPath = output;
    }
    return true;
}

Roots MakeRoots(
    const std::filesystem::path& dataRoot,
    const std::filesystem::path& authoredRoot,
    const std::filesystem::path& localSavedRoot,
    const std::filesystem::path& localDataRoot) {
    const auto localData =
        (localDataRoot.empty() ? dataRoot : localDataRoot).lexically_normal();
    const auto workspace = authoredRoot.empty()
                               ? ResolveAuthoredWorkspaceDirectory(localData)
                               : authoredRoot;
    return {
        .dataRoot = dataRoot.lexically_normal(),
        .localDataRoot = localData,
        .authoredRoot = workspace.lexically_normal(),
        .localRenderRoot =
            ((localSavedRoot.empty() ? LocalSavedDirectory(localData)
                                     : localSavedRoot) /
             "renders" / "Invisible Places")
                .lexically_normal(),
    };
}

std::filesystem::path MakeDataPathPortable(
    const std::filesystem::path& path,
    const Roots& roots) {
    const auto primary = PortableUnderRoot(path, roots.dataRoot, kDataToken);
    if (primary != path || roots.localDataRoot.empty() ||
        roots.localDataRoot == roots.dataRoot) {
        return primary;
    }
    return PortableUnderRoot(path, roots.localDataRoot, kDataToken);
}

std::filesystem::path ResolveDataPath(
    const std::filesystem::path& path,
    const Roots& roots) {
    if (path.empty()) {
        return path;
    }
    const auto resolveRelative = [&](const std::filesystem::path& relative) {
        const auto primary = (roots.dataRoot / relative).lexically_normal();
        std::error_code existsError;
        if (std::filesystem::exists(primary, existsError) ||
            roots.localDataRoot.empty() ||
            roots.localDataRoot == roots.dataRoot) {
            return primary;
        }
        const auto local =
            (roots.localDataRoot / relative).lexically_normal();
        existsError.clear();
        return std::filesystem::exists(local, existsError) && !existsError
                   ? local
                   : primary;
    };
    if (HasToken(path, kDataToken)) {
        return resolveRelative(RemoveFirstComponent(path));
    }
    if (!path.is_absolute()) {
        const auto components = Components(path);
        if (!components.empty() &&
            LowercaseAscii(components.front().string()) == "data") {
            return resolveRelative(JoinComponents(components, 1U));
        }
        return resolveRelative(path);
    }
    std::filesystem::path relative;
    if (LexicallyWithin(path, roots.dataRoot, &relative)) {
        return path.lexically_normal();
    }
    if (LexicallyWithin(path, roots.localDataRoot, &relative)) {
        return path.lexically_normal();
    }
    const auto components = Components(path);
    if (const auto data = FindComponent(components, "Data");
        data.has_value() && data.value() + 1U < components.size()) {
        return resolveRelative(
            JoinComponents(components, data.value() + 1U));
    }
    return path.lexically_normal();
}

std::filesystem::path MakeWorkspacePathPortable(
    const std::filesystem::path& path,
    const Roots& roots) {
    return PortableUnderRoot(path, roots.authoredRoot, kWorkspaceToken);
}

std::filesystem::path ResolveWorkspacePath(
    const std::filesystem::path& path,
    const Roots& roots) {
    if (path.empty()) {
        return path;
    }
    if (HasToken(path, kWorkspaceToken)) {
        return ResolveToken(path, roots.authoredRoot, kWorkspaceToken);
    }
    const auto components = Components(path);
    if (!path.is_absolute()) {
        if (!components.empty() &&
            LowercaseAscii(components.front().string()) == "saved") {
            return (roots.authoredRoot / JoinComponents(components, 1U))
                .lexically_normal();
        }
        return (roots.authoredRoot / path).lexically_normal();
    }
    std::filesystem::path relative;
    if (LexicallyWithin(path, roots.authoredRoot, &relative)) {
        return path.lexically_normal();
    }
    if (const auto saved = FindComponent(components, "Saved");
        saved.has_value() && saved.value() + 1U < components.size()) {
        return (roots.authoredRoot /
                JoinComponents(components, saved.value() + 1U))
            .lexically_normal();
    }
    if (const auto animations = FindComponent(components, "animations");
        animations.has_value()) {
        return (roots.authoredRoot /
                JoinComponents(components, animations.value()))
            .lexically_normal();
    }
    return path.lexically_normal();
}

std::filesystem::path MakeRenderPathPortable(
    const std::filesystem::path& path,
    const Roots& roots) {
    return PortableUnderRoot(path, roots.localRenderRoot, kRenderToken);
}

std::filesystem::path ResolveRenderPath(
    const std::filesystem::path& path,
    const Roots& roots) {
    if (path.empty()) {
        return path;
    }
    if (HasToken(path, kRenderToken)) {
        return ResolveToken(path, roots.localRenderRoot, kRenderToken);
    }
    const auto components = Components(path);
    if (!path.is_absolute()) {
        if (!components.empty() &&
            LowercaseAscii(components.front().string()) == "renders") {
            return (roots.localRenderRoot / JoinComponents(components, 1U))
                .lexically_normal();
        }
        return (roots.localRenderRoot / path).lexically_normal();
    }
    std::filesystem::path relative;
    if (LexicallyWithin(path, roots.localRenderRoot, &relative)) {
        return path.lexically_normal();
    }
    if (const auto renders = FindComponent(components, "renders");
        renders.has_value()) {
        auto first = renders.value() + 1U;
        if (first < components.size()) {
            const auto legacyFolder =
                LowercaseAscii(components[first].string());
            if (legacyFolder == "invisible places" ||
                legacyFolder == "videos") {
                ++first;
            }
        }
        return (roots.localRenderRoot / JoinComponents(components, first))
            .lexically_normal();
    }
    return path.lexically_normal();
}

void MakeAnimationPathPortable(
    invisible_places::camera::AnimationPath* path,
    const Roots& roots) {
    if (path == nullptr) {
        return;
    }
    TransformAssociations(
        &path->associatedLayerPaths,
        [&](const auto& value) { return MakeDataPathPortable(value, roots); });
    path->exportSettings.outputDirectory = MakeRenderPathPortable(
        path->exportSettings.outputDirectory,
        roots).string();
}

void ResolveAnimationPath(
    invisible_places::camera::AnimationPath* path,
    const Roots& roots) {
    if (path == nullptr) {
        return;
    }
    TransformAssociations(
        &path->associatedLayerPaths,
        [&](const auto& value) { return ResolveDataPath(value, roots); });
    path->exportSettings.outputDirectory = ResolveRenderPath(
        path->exportSettings.outputDirectory,
        roots).string();
}

void MakeProjectDocumentPortable(
    invisible_places::serialization::ProjectDocument* document,
    const Roots& roots) {
    TransformProject(
        document,
        [&](const auto& value) { return MakeDataPathPortable(value, roots); },
        [&](const auto& value) {
            return MakeWorkspacePathPortable(value, roots);
        },
        [&](const auto& value) { return MakeRenderPathPortable(value, roots); });
    if (document == nullptr) {
        return;
    }
    // Cache payloads and their source fingerprints are derived from each
    // machine's local PLY files. Keeping them in cloud-authored JSON would
    // make the workspace large and can incorrectly validate another
    // computer's cache. Authored Water state remains and rebuilds locally.
    for (auto& group : document->scenePointCloudGroups) {
        group.waterSurfaceCache.reset();
    }
    document->waterPathCache.reset();
    document->waterPathCacheManifest.reset();
    document->waterRippleRuntimeCaches.clear();
    for (auto& state : document->waterSceneStates) {
        state.pathCache.reset();
        state.pathCacheManifest.reset();
        state.rippleRuntimeCaches.clear();
    }
}

void ResolveProjectDocument(
    invisible_places::serialization::ProjectDocument* document,
    const Roots& roots) {
    TransformProject(
        document,
        [&](const auto& value) { return ResolveDataPath(value, roots); },
        [&](const auto& value) { return ResolveWorkspacePath(value, roots); },
        [&](const auto& value) { return ResolveRenderPath(value, roots); });
}

void MakeWaterSourcesDocumentPortable(
    invisible_places::serialization::WaterSourcesDocument* document,
    const Roots& roots) {
    TransformWaterSources(
        document,
        [&](const auto& value) { return MakeDataPathPortable(value, roots); });
    if (document != nullptr) {
        document->pathCache.reset();
        document->rippleRuntimeCaches.clear();
    }
}

void ResolveWaterSourcesDocument(
    invisible_places::serialization::WaterSourcesDocument* document,
    const Roots& roots) {
    TransformWaterSources(
        document,
        [&](const auto& value) { return ResolveDataPath(value, roots); });
}

void MakeRenderSetupDocumentPortable(
    invisible_places::serialization::RenderSetupDocument* document,
    const Roots& roots) {
    if (document == nullptr) {
        return;
    }
    document->outputPath = MakeRenderPathPortable(document->outputPath, roots);
    document->logPath = MakeRenderPathPortable(document->logPath, roots);
    document->sourceProjectPath =
        MakeWorkspacePathPortable(document->sourceProjectPath, roots);
    document->originalAnimationPath =
        MakeWorkspacePathPortable(document->originalAnimationPath, roots);
    MakeAnimationPathPortable(&document->animation, roots);
    document->exportPreset.settings.outputDirectory =
        MakeRenderPathPortable(
            document->exportPreset.settings.outputDirectory,
            roots).string();
    MakeWaterSourcesDocumentPortable(&document->authoredWater, roots);
    for (auto& fingerprint : document->sourceFingerprints) {
        fingerprint.sourcePath =
            MakeDataPathPortable(fingerprint.sourcePath, roots);
    }
}

void ResolveRenderSetupDocument(
    invisible_places::serialization::RenderSetupDocument* document,
    const Roots& roots) {
    if (document == nullptr) {
        return;
    }
    document->outputPath = ResolveRenderPath(document->outputPath, roots);
    document->logPath = ResolveRenderPath(document->logPath, roots);
    document->sourceProjectPath =
        ResolveWorkspacePath(document->sourceProjectPath, roots);
    document->originalAnimationPath =
        ResolveWorkspacePath(document->originalAnimationPath, roots);
    ResolveAnimationPath(&document->animation, roots);
    document->exportPreset.settings.outputDirectory =
        ResolveRenderPath(
            document->exportPreset.settings.outputDirectory,
            roots).string();
    ResolveWaterSourcesDocument(&document->authoredWater, roots);
    for (auto& fingerprint : document->sourceFingerprints) {
        fingerprint.sourcePath = ResolveDataPath(fingerprint.sourcePath, roots);
    }
}

}  // namespace invisible_places::app::workspace
