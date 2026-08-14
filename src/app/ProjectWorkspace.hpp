#pragma once

#include "camera/AnimationPath.hpp"
#include "serialization/ProjectDocument.hpp"
#include "serialization/RenderSetupDocument.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace invisible_places::app::workspace {

inline constexpr const char* kWorkspaceEnvironmentVariable =
    "INVISIBLE_PLACES_WORKSPACE_DIR";
inline constexpr const char* kWorkspaceMarkerFilename =
    ".invisible_places-workspace";
inline constexpr const char* kSharedDataEnvironmentVariable =
    "INVISIBLE_PLACES_SHARED_DATA_DIR";
inline constexpr const char* kSharedDataMarkerFilename =
    ".invisible_places-data-workspace";

struct FileRevision {
    bool exists = false;
    std::uint64_t sizeBytes = 0U;
    std::uint64_t contentHash = 0U;

    friend bool operator==(const FileRevision&, const FileRevision&) = default;
};

// Project files are shared through OneDrive, where two machines can save
// independent authoring changes before either sees the other's write.  These
// types describe a semantic three-way merge: object fields and identifiable
// array entries merge independently, while only overlapping scalar edits are
// returned for an explicit user choice.
enum class JsonMergeSide {
    Local,
    Remote,
};

struct JsonMergeConflict {
    std::string path;
    std::optional<nlohmann::json> baseValue;
    std::optional<nlohmann::json> localValue;
    std::optional<nlohmann::json> remoteValue;
};

struct JsonMergeResult {
    nlohmann::json merged;
    std::vector<JsonMergeConflict> conflicts;
};

[[nodiscard]] JsonMergeResult MergeJsonDocuments(
    const nlohmann::json& baseline,
    const nlohmann::json& local,
    const nlohmann::json& remote,
    const std::unordered_map<std::string, JsonMergeSide>& resolutions = {});
[[nodiscard]] std::optional<nlohmann::json> ReadJsonDocument(
    const std::filesystem::path& path,
    std::string* errorMessage = nullptr);
bool WriteJsonDocument(
    const nlohmann::json& document,
    const std::filesystem::path& path,
    std::string* errorMessage = nullptr);

struct Roots {
    std::filesystem::path dataRoot;
    std::filesystem::path localDataRoot;
    std::filesystem::path authoredRoot;
    std::filesystem::path localRenderRoot;
};

[[nodiscard]] std::filesystem::path LocalSavedDirectory(
    const std::filesystem::path& dataRoot);
[[nodiscard]] std::filesystem::path WorkspaceMarkerPath(
    const std::filesystem::path& dataRoot);
[[nodiscard]] std::filesystem::path SharedDataMarkerPath(
    const std::filesystem::path& localDataRoot);
[[nodiscard]] std::filesystem::path ResolveSharedDataDirectory(
    const std::filesystem::path& localDataRoot,
    const std::filesystem::path& requestedDataRoot = {});
[[nodiscard]] std::filesystem::path ResolveAuthoredWorkspaceDirectory(
    const std::filesystem::path& dataRoot);
[[nodiscard]] bool WorkspaceEnvironmentOverrideActive();
[[nodiscard]] bool SharedDataEnvironmentOverrideActive();
bool WriteWorkspaceMarker(
    const std::filesystem::path& dataRoot,
    const std::filesystem::path& authoredRoot,
    std::string* errorMessage);
bool WriteSharedDataMarker(
    const std::filesystem::path& localDataRoot,
    const std::filesystem::path& sharedDataRoot,
    std::string* errorMessage);

[[nodiscard]] std::optional<FileRevision> ReadFileRevision(
    const std::filesystem::path& path,
    std::string* errorMessage = nullptr);
[[nodiscard]] bool FileRevisionMatches(
    const std::filesystem::path& path,
    const FileRevision& expected,
    std::string* errorMessage = nullptr);
[[nodiscard]] std::filesystem::path ConflictRecoveryDirectory(
    const std::filesystem::path& localSavedRoot);
bool PreserveConflictRecoveryCopy(
    const std::filesystem::path& stagedPath,
    const std::filesystem::path& intendedTarget,
    const std::filesystem::path& localSavedRoot,
    std::filesystem::path* recoveryPath,
    std::string* errorMessage = nullptr);

[[nodiscard]] Roots MakeRoots(
    const std::filesystem::path& dataRoot,
    const std::filesystem::path& authoredRoot = {},
    const std::filesystem::path& localSavedRoot = {},
    const std::filesystem::path& localDataRoot = {});

[[nodiscard]] std::filesystem::path MakeDataPathPortable(
    const std::filesystem::path& path,
    const Roots& roots);
[[nodiscard]] std::filesystem::path ResolveDataPath(
    const std::filesystem::path& path,
    const Roots& roots);
[[nodiscard]] std::filesystem::path MakeWorkspacePathPortable(
    const std::filesystem::path& path,
    const Roots& roots);
[[nodiscard]] std::filesystem::path ResolveWorkspacePath(
    const std::filesystem::path& path,
    const Roots& roots);
[[nodiscard]] std::filesystem::path MakeRenderPathPortable(
    const std::filesystem::path& path,
    const Roots& roots);
[[nodiscard]] std::filesystem::path ResolveRenderPath(
    const std::filesystem::path& path,
    const Roots& roots);

void MakeAnimationPathPortable(
    invisible_places::camera::AnimationPath* path,
    const Roots& roots);
void ResolveAnimationPath(
    invisible_places::camera::AnimationPath* path,
    const Roots& roots);
void MakeProjectDocumentPortable(
    invisible_places::serialization::ProjectDocument* document,
    const Roots& roots);
void ResolveProjectDocument(
    invisible_places::serialization::ProjectDocument* document,
    const Roots& roots);
void MakeWaterSourcesDocumentPortable(
    invisible_places::serialization::WaterSourcesDocument* document,
    const Roots& roots);
void ResolveWaterSourcesDocument(
    invisible_places::serialization::WaterSourcesDocument* document,
    const Roots& roots);
void MakeRenderSetupDocumentPortable(
    invisible_places::serialization::RenderSetupDocument* document,
    const Roots& roots);
void ResolveRenderSetupDocument(
    invisible_places::serialization::RenderSetupDocument* document,
    const Roots& roots);

}  // namespace invisible_places::app::workspace
