#pragma once

#include <unordered_set>
#include <glim/mapping/global_mapping.hpp>
#include <glim/mapping/async_global_mapping.hpp>
#include <glim/viewer/interactive_viewer.hpp>

namespace guik {
class ProgressModal;
class ProgressInterface;
}  // namespace guik

namespace glim {

class OfflineViewer : public InteractiveViewer {
public:
  OfflineViewer(const std::string& init_map_path = "", bool init_map_optimize = false, const std::string& merge_base_name = "", const std::string& browse_dir = "");
  virtual ~OfflineViewer() override;

private:
  virtual void setup_ui() override;
  // Draws the Save Merged Map button below Optimize in the Selection window.
  virtual void selection_ui_extra() override;
  virtual bool show_graph_controls() const override { return false; }
  virtual bool show_log_panel() const override { return false; }
  virtual bool skip_session_merge_prompt() const override { return true; }
  virtual bool simple_loop_close_ui() const override { return true; }

  void main_menu();

  // Next unused `<base>_merged[_N]` sibling of the first-loaded map, which is
  // what the Save button writes to. Empty if no map is loaded.
  std::string next_merged_save_path() const;

  std::shared_ptr<GlobalMapping> load_map(guik::ProgressInterface& progress, const std::string& path, std::shared_ptr<GlobalMapping> global_mapping, bool skip_optimize_prompt);
  bool save_map(guik::ProgressInterface& progress, const std::string& path);
  bool export_map(guik::ProgressInterface& progress, const std::string& path);

private:
  std::string init_map_path;
  // Skip the "Do optimization?" prompt for the auto-opened map and answer yes.
  bool init_map_optimize;
  // Path of the first map loaded this session; the merged-name base is derived
  // from it, so later "Open Additional Map" loads do not move the target.
  std::string first_loaded_map_path;
  // Base map name for the merged save target, supplied by the GUI. Empty
  // means derive it from the loaded map's directory name.
  std::string merge_base_name;
  // Set by the Save Merged Map button, consumed on the next main_menu() pass
  // (that is where progress_modal's "save" slot is driven from).
  bool save_merged_requested = false;
  // Destination of the Save Merged Map save currently in flight; announced on
  // stdout once it succeeds so the mapping GUI can point its output folder at it.
  std::string pending_merged_save_path;
  // Start folder for File > Open dialogs (see open_dialog_start_dir).
  std::string browse_dir;
  std::string launch_map_path;
  std::unique_ptr<guik::ProgressModal> progress_modal;

  std::unordered_set<std::string> imported_shared_libs;
  std::unique_ptr<AsyncGlobalMapping> async_global_mapping;
};

}  // namespace glim
