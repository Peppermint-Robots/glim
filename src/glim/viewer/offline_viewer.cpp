#include <glim/viewer/offline_viewer.hpp>

#include <boost/filesystem.hpp>
#include <gtsam_points/config.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#include <glim/util/config.hpp>

#include <spdlog/spdlog.h>
#include <iostream>
#include <portable-file-dialogs.h>
#include <glk/io/ply_io.hpp>
#include <guik/recent_files.hpp>
#include <guik/progress_modal.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

// Folder the Open dialogs start in: --browse_dir if given, else the parent of
// --map_path (maps live in <site>/3D_maps/<name>, so that is 3D_maps), else
// the most recently opened folder.
static std::string open_dialog_start_dir(const std::string& browse_dir, const std::string& map_path, const std::string& recent) {
  if (!browse_dir.empty() && boost::filesystem::is_directory(browse_dir)) {
    return browse_dir;
  }
  if (!map_path.empty()) {
    boost::filesystem::path p(map_path);
    if (p.filename() == ".") {
      p = p.parent_path();
    }
    if (boost::filesystem::is_directory(p.parent_path())) {
      return p.parent_path().string();
    }
  }
  return recent;
}

OfflineViewer::OfflineViewer(const std::string& init_map_path, bool init_map_optimize, const std::string& merge_base_name, const std::string& browse_dir)
: init_map_path(init_map_path),
  init_map_optimize(init_map_optimize),
  merge_base_name(merge_base_name),
  browse_dir(browse_dir),
  launch_map_path(init_map_path) {}

OfflineViewer::~OfflineViewer() {}

void OfflineViewer::setup_ui() {
  auto viewer = guik::LightViewer::instance();
  viewer->register_ui_callback("main_menu", [this] { main_menu(); });

  progress_modal.reset(new guik::ProgressModal("offline_viewer_progress"));

#ifdef GTSAM_POINTS_USE_CUDA
  gtsam_points::LinearizationHook::register_hook([] { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif
}

void OfflineViewer::selection_ui_extra() {
  // Sits directly below Optimize in the Selection window, mirroring the Map
  // Editor's in-panel Save: one click, no folder picker.
  ImGui::Separator();

  const bool has_map = static_cast<bool>(async_global_mapping);
  if (!has_map) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("Save Merged Map")) {
    save_merged_requested = true;
  }
  if (!has_map) {
    ImGui::EndDisabled();
  }

  const std::string merged_path = next_merged_save_path();
  if (ImGui::IsItemHovered() && !merged_path.empty()) {
    ImGui::SetTooltip("Save to %s", merged_path.c_str());
  }
  if (!merged_path.empty()) {
    ImGui::Text("-> %s", boost::filesystem::path(merged_path).filename().string().c_str());
  }
}

void OfflineViewer::main_menu() {
  bool start_open_map = false || !init_map_path.empty();
  bool start_close_map = false;
  bool start_save_map = false;
  bool start_export_map = false;
  // Set by the Save Merged Map button in the Selection window, which runs in
  // a different ui callback, so it arrives via the member flag.
  const bool start_save_merged = save_merged_requested;
  save_merged_requested = false;
  const std::string merged_path = start_save_merged ? next_merged_save_path() : std::string();

  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (!async_global_mapping) {  // if a previously loaded map does not yet exist
        if (ImGui::MenuItem("Open New Map")) {
          start_open_map = true;
        }
      } else {
        if (ImGui::MenuItem("Open Additional Map")) {
          start_open_map = true;
        }
      }

      if (ImGui::MenuItem("Close Map")) {
        if (pfd::message("Warning", "Close the map?").result() == pfd::button::ok) {
          start_close_map = true;
        }
      }

      if (ImGui::BeginMenu("Save")) {
        if (ImGui::MenuItem("Save Map")) {
          start_save_map = true;
        }

        if (ImGui::MenuItem("Export Points")) {
          start_export_map = true;
        }

        ImGui::EndMenu();
      }

      if (ImGui::MenuItem("Quit")) {
        if (pfd::message("Warning", "Quit?").result() == pfd::button::ok) {
          request_to_terminate = true;
        }
      }

      ImGui::EndMenu();
    }

    ImGui::EndMainMenuBar();
  }

  // open map
  if (start_open_map) {
    logger->debug("open map");
    std::string map_path;

    guik::RecentFiles recent_files("offline_viewer_open");
    // True only for the map auto-opened from --map_path: that one may skip the
    // "Do optimization?" prompt. Maps picked from the File menu never do.
    bool auto_opened = false;
    if (init_map_path.empty()) {
      map_path = pfd::select_folder("Select a dump directory", open_dialog_start_dir(browse_dir, launch_map_path, recent_files.most_recent())).result();
    } else {
      map_path = init_map_path;
      init_map_path.clear();
      auto_opened = true;
    }

    if (!map_path.empty()) {
      logger->debug("open map from {}", map_path);
      recent_files.push(map_path);

      if (boost::filesystem::exists(map_path + "/config")) {
        logger->info("Use config from {}", map_path + "/config");
        GlobalConfig::instance(map_path + "/config", true);
      } else {
        logger->warn("No config found in {}", map_path);
      }

      const Config config_ros(GlobalConfig::get_config_path("config_ros"));
      const std::vector<std::string> ext_module_names = config_ros.param<std::vector<std::string>>("glim_ros", "extension_modules", {});
      for (const auto& name : ext_module_names) {
        if (name.find("viewer") != std::string::npos || name.find("monitor") != std::string::npos) {
          continue;
        }
        if (imported_shared_libs.count(name)) {
          logger->debug("Extension module {} already loaded", name);
          continue;
        }

        logger->info("Export classes from {}", name);
        ExtensionModule::export_classes(name);
        imported_shared_libs.insert(name);
      }

      // if a map is already loaded, use existing map to load new map into
      std::shared_ptr<GlobalMapping> global_mapping;
      if (async_global_mapping) {
        logger->info("global map already exists, loading new map into existing global map");
        global_mapping = std::dynamic_pointer_cast<GlobalMapping>(async_global_mapping->get_global_mapping());
      }

      if (first_loaded_map_path.empty()) {
        first_loaded_map_path = map_path;
      }

      const bool skip_optimize_prompt = auto_opened && init_map_optimize;
      progress_modal->open<std::shared_ptr<GlobalMapping>>("open", [this, map_path, global_mapping, skip_optimize_prompt](guik::ProgressInterface& progress) {
        return load_map(progress, map_path, global_mapping, skip_optimize_prompt);
      });
    }
  }
  auto open_result = progress_modal->run<std::shared_ptr<GlobalMapping>>("open");
  if (open_result) {
    if (!(*open_result)) {
      pfd::message("Error", "Failed to load map").result();
    } else {
      async_global_mapping.reset(new glim::AsyncGlobalMapping(*open_result, 1e6));
    }
  }

  // save merged map -- fixed destination, so no picker
  if (start_save_merged) {
    if (merged_path.empty()) {
      logger->warn("No map loaded, nothing to save");
    } else {
      logger->info("Saving merged map to {}", merged_path);
      // Overwriting a previous merge: clear it first. Submaps are written as
      // numbered dirs (000000, 000001, ...), so a shorter merge left on top of
      // a longer one would otherwise keep the stale trailing submaps.
      boost::system::error_code ec;
      if (boost::filesystem::exists(merged_path)) {
        logger->info("Removing existing {}", merged_path);
        boost::filesystem::remove_all(merged_path, ec);
        if (ec) {
          logger->error("Failed to clear {}: {}", merged_path, ec.message());
        }
      }
      boost::filesystem::create_directories(merged_path);
      guik::RecentFiles recent_files("offline_viewer_save");
      recent_files.push(merged_path);
      const std::string path = merged_path;
      pending_merged_save_path = merged_path;
      progress_modal->open<bool>("save", [this, path](guik::ProgressInterface& progress) { return save_map(progress, path); });
    }
  }

  // save map
  if (start_save_map) {
    if (!async_global_mapping) {
      logger->warn("No map data to save");
    } else {
      guik::RecentFiles recent_files("offline_viewer_save");
      const std::string path = pfd::select_folder("Select a directory to save the map", recent_files.most_recent()).result();
      if (!path.empty()) {
        recent_files.push(path);
        progress_modal->open<bool>("save", [this, path](guik::ProgressInterface& progress) { return save_map(progress, path); });
      }
    }
  }
  auto save_result = progress_modal->run<bool>("save");
  if (save_result && !pending_merged_save_path.empty()) {
    if (*save_result) {
      // Machine-readable line for mapping_tool.py, which tails this process's
      // log. Plain stdout (not spdlog) so it carries no colour codes/prefix.
      std::cout << "[MERGED_MAP_SAVED] " << pending_merged_save_path << std::endl;
    }
    pending_merged_save_path.clear();
  }

  // export map
  if (start_export_map) {
    guik::RecentFiles recent_files("offline_viewer_export");
    const std::string path = pfd::save_file("Select the file destination", recent_files.most_recent(), {"PLY", "*.ply"}).result();
    if (!path.empty()) {
      recent_files.push(path);
      progress_modal->open<bool>("export", [this, path](guik::ProgressInterface& progress) { return export_map(progress, path); });
    }
  }
  auto export_result = progress_modal->run<bool>("export");

  // close map
  if (start_close_map) {
    if (async_global_mapping) {
      logger->info("Closing map");
      async_global_mapping->join();
      async_global_mapping.reset();
      first_loaded_map_path.clear();
      clear();
    } else {
      logger->warn("No map to close");
    }
  }
}

namespace {

// True if `stem` looks like "<base>_merged" or "<base>_merged_N", and if so
// writes the "<base>" part to `base_out`.
bool split_merged_name(const std::string& stem, const std::string& tag, std::string* base_out) {
  const auto tag_pos = stem.rfind(tag);
  if (tag_pos == std::string::npos) {
    return false;
  }

  const std::string rest = stem.substr(tag_pos + tag.size());
  bool merged = rest.empty();
  if (!merged && rest[0] == '_') {
    const std::string digits = rest.substr(1);
    merged = !digits.empty() && digits.find_first_not_of("0123456789") == std::string::npos;
  }

  if (merged && base_out) {
    *base_out = stem.substr(0, tag_pos);
  }
  return merged;
}

}  // namespace

std::string OfflineViewer::next_merged_save_path() const {
  if (first_loaded_map_path.empty()) {
    return "";
  }

  static const std::string tag = "_merged";

  // Strip a trailing slash so filename() yields the directory name, not "".
  boost::filesystem::path src(first_loaded_map_path);
  if (src.filename() == ".") {
    src = src.parent_path();
  }
  const boost::filesystem::path parent = src.parent_path();
  const std::string loaded = src.filename().string();

  // Was the map loaded first itself a merge result? That is what separates
  // chaining a further merge onto it (which must not clobber its input) from
  // simply redoing a merge of the original maps (which should overwrite the
  // stale output).
  std::string merged_base;
  const bool loaded_is_merged = split_merged_name(loaded, tag, &merged_base);

  std::string stem;
  if (loaded_is_merged) {
    stem = merged_base;
  } else if (!merge_base_name.empty()) {
    // --merge_base is authoritative: the GUI knows the real base map name, so
    // a map legitimately named e.g. "floor_2" is not mistaken for a segment.
    stem = merge_base_name;
  } else if (const auto us = loaded.rfind('_');
             us != std::string::npos && us + 1 < loaded.size() &&
             loaded.find_first_not_of("0123456789", us + 1) == std::string::npos) {
    // Break segment "<base>_N" collapses to "<base>" so that loading map or
    // map_1 first agrees on one target. Ambiguous for a name that genuinely
    // ends in _<digits> -- pass --merge_base to settle it.
    stem = loaded.substr(0, us);
  } else {
    stem = loaded;
  }

  const boost::filesystem::path plain = parent / (stem + tag);

  // Re-merging the original maps: reuse "<base>_merged" and overwrite whatever
  // stale output is sitting there, rather than piling up _1, _2, ...
  if (!loaded_is_merged) {
    return plain.string();
  }

  // Chaining onto an already-merged map: the loaded map is the input, so step
  // past it to the first free name in the _merged, _merged_1, ... sequence.
  boost::filesystem::path candidate = plain;
  for (int n = 1; boost::filesystem::exists(candidate); ++n) {
    candidate = parent / (stem + tag + "_" + std::to_string(n));
  }
  return candidate.string();
}

std::shared_ptr<glim::GlobalMapping> OfflineViewer::load_map(
  guik::ProgressInterface& progress,
  const std::string& path,
  std::shared_ptr<GlobalMapping> global_mapping,
  bool skip_optimize_prompt) {
  progress.set_title("Load map");
  progress.set_text("Now loading");
  progress.set_maximum(1);

  if (global_mapping == nullptr) {  // if no map is loaded yet initialize new GlobalMapping
    glim::GlobalMappingParams params;
    params.isam2_relinearize_skip = 1;
    params.isam2_relinearize_thresh = 0.0;

    if (skip_optimize_prompt) {
      params.enable_optimization = true;
    } else {
      const auto result = pfd::message("Confirm", "Do optimization?", pfd::choice::yes_no).result();
      params.enable_optimization = (result == pfd::button::ok) || (result == pfd::button::yes);
    }

    logger->info("enable_optimization={}", params.enable_optimization);
    global_mapping.reset(new glim::GlobalMapping(params));
  }

  if (!global_mapping->load(path)) {
    logger->error("failed to load {}", path);
    return nullptr;
  }

  return global_mapping;
}

bool OfflineViewer::save_map(guik::ProgressInterface& progress, const std::string& path) {
  progress.set_title("Save map");
  progress.set_text("Now saving");
  async_global_mapping->save(path);
  return true;
}

bool OfflineViewer::export_map(guik::ProgressInterface& progress, const std::string& path) {
  progress.set_title("Export points");
  progress.set_text("Concatenating submaps");
  progress.set_maximum(3);
  progress.increment();
  auto points = async_global_mapping->export_points();

  if (!points || !points->has_points()) {
    logger->warn("No points available for export");
    return false;
  }

  progress.set_text("Writing to file");
  progress.increment();

  glk::PLYData ply;
  ply.vertices.reserve(points->size());
  const bool has_intensities = points->has_intensities();
  if (has_intensities) {
    ply.intensities.reserve(points->size());
  }

  for (size_t i = 0; i < points->size(); ++i) {
    ply.vertices.push_back(points->points[i].head<3>().cast<float>());
    if (has_intensities) {
      ply.intensities.push_back(points->intensities[i]);
    }
  }

  glk::save_ply_binary(path, ply);
  return true;
}

}  // namespace glim