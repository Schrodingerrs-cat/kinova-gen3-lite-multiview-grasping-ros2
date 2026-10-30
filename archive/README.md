# Archive: the earlier, non-ROS version

Not used by the ROS 2 pipeline; kept for reference. Paths inside these scripts assume the old layout and are not maintained.

* `kortex_api_tools/` - Kortex Python API tools: hand-eye calibration (`hand_eye.py`), building the fixed scan views from IK (`make_views.py`), link check, the report generator, the table-based hand-eye refinement experiment.
* `old_calibration/` - earlier hand-eye / views / home-pose files (the original camera mounting) and the calibration capture images.
* `config.prev.yaml`, `place.py`, `README_kortex_version.md` - earlier config, free-slot placing (unused), earlier README.

To use these tools again, copy them back next to `common.py` (the calibration ones to `calibration/`), connect with the Kortex API, and keep the ROS driver stopped.
