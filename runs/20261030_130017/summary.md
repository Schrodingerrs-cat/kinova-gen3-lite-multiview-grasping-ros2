# Run 20261030_130017

**Result:** Done: 2 object(s) picked and put back from one scan

```
== Scan: fixed viewpoints, one point cloud each (done once) ==
  scan pose reached (joint-space move to the saved scan joints)
  view 1/10 obl_near_left: 83308 points
  view 2/10 left_45: 97978 points
  view 3/10 obl_left: 82214 points
  view 4/10 obl_near: 88031 points
  view 5/10 left_22: 97993 points
  view 6/10 center: 95524 points
  view 7/10 right_22: 97491 points
  view 8/10 obl_right: 93783 points
  view 9/10 obl_near_right: 96149 points
  view 10/10 right_45: 96105 points
== Object 1: perception ==
[1.1] scans: ['scan_0.npz', 'scan_1.npz', 'scan_2.npz', 'scan_3.npz', 'scan_4.npz', 'scan_5.npz', 'scan_6.npz', 'scan_7.npz', 'scan_8.npz', 'scan_9.npz']
[1.1] table of view 0 tilted 7.29 deg, offset +24.7 mm vs the centre view: aligned
[1.1] table of view 1 tilted 3.57 deg, offset +37.8 mm vs the centre view: aligned
[1.1] table of view 2 tilted 6.72 deg, offset +19.2 mm vs the centre view: aligned
[1.1] table of view 3 tilted 2.50 deg, offset +8.2 mm vs the centre view: aligned
[1.1] table of view 4 tilted 1.43 deg, offset +38.4 mm vs the centre view: aligned
[1.1] table of view 6 tilted 1.42 deg, offset -3.1 mm vs the centre view: aligned
[1.1] table of view 7 tilted 4.09 deg, offset +11.5 mm vs the centre view: aligned
[1.1] table of view 8 tilted 2.68 deg, offset +2.8 mm vs the centre view: aligned
[1.1] table of view 9 tilted 3.12 deg, offset +6.0 mm vs the centre view: aligned
[1.1] ICP scan 0 -> scan 5: shift 2.8 mm, rot 0.61 deg, trimmed rms 4.15 mm
[1.1] ICP scan 1 -> scan 5: shift 8.3 mm, rot 1.37 deg, trimmed rms 3.84 mm
[1.1] ICP scan 2 -> scan 5: shift 18.7 mm, rot 2.70 deg, trimmed rms 4.51 mm
[1.1] ICP scan 3 -> scan 5: shift 2.5 mm, rot 0.43 deg, trimmed rms 3.67 mm
[1.1] ICP scan 4 -> scan 5: shift 8.7 mm, rot 1.44 deg, trimmed rms 4.87 mm
[1.1] ICP scan 6 -> scan 5: shift 4.6 mm, rot 0.89 deg, trimmed rms 3.12 mm
[1.1] ICP scan 7 -> scan 5: shift 5.2 mm, rot 0.85 deg, trimmed rms 4.23 mm
[1.1] ICP scan 8 -> scan 5: shift 7.0 mm, rot 0.72 deg, trimmed rms 3.77 mm
[1.1] ICP scan 9 -> scan 5: shift 5.4 mm, rot 1.11 deg, trimmed rms 3.11 mm
[1.0] 10 viewpoints: obl_near_left (camera z 0.341 m), left_45 (camera z 0.345 m), obl_left (camera z 0.344 m), obl_near (camera z 0.339 m), left_22 (camera z 0.345 m), center (camera z 0.345 m), right_22 (camera z 0.345 m), obl_right (camera z 0.342 m), obl_near_right (camera z 0.340 m), right_45 (camera z 0.345 m)
[1.1] merged 928576 -> 213621 points (voxel 0.004 m)
[1.2] workspace: 112440 points
[1.3] plane normal=(-0.078,-0.034,0.996) -> 77226 inliers, z at x,y=0: -0.0556 m
[1.4] object points: 10955 (> 12 mm above plane; 5781 noise points and 2298 single-view points removed)
[1.5] 2 cluster(s):
      cluster 0: 6109 pts, extent [0.131 0.211 0.076], top 79 mm above table, centroid [0.283 0.227 0.015]
      cluster 1: 2241 pts, extent [0.113 0.109 0.061], top 70 mm above table, centroid [ 0.503 -0.075  0.011]
[2.1] centroids of all objects: object 0: (0.283, 0.227, 0.015); object 1: (0.503, -0.075, 0.011)
      target = object 0 of 2 on the table (nearest-first order)
[2.1] centroid = [0.2829 0.2272 0.0148], object height 72 mm above the table
[2.2] 6109 normals; mean angle to the centroid direction = 57.8 deg
[2.3] antipodal pair rejected: its midpoint is 26 mm from the centroid (limit 25 mm), an off-centre feature
[2.3] no usable antipodal pair; grasping across the body near its middle at (0.283, 0.225), local body direction [-0.5   0.86], width 72 mm
[3] PCA eigenvalues [0.000282 0.001004 0.001762], major axis [-0.128 -0.992  0.016]; approach [ 0.  0. -1.] via vertical (grasping across the body)
  target at (0.283, 0.227) of 2 object(s)
== Object 1: grasp plan check (no motion) ==
pre_grasp: tool xyz [0.283 0.225 0.138] theta [-180.     0.  -149.7] | bounds ok | IK max joint step 102 deg -> OK
grasp    : tool xyz [0.283 0.225 0.038] theta [-180.     0.  -149.7] | bounds ok | IK max joint step 19 deg -> OK
lift     : tool xyz [0.283 0.225 0.158] theta [-180.     0.  -149.7] | bounds ok | IK max joint step 23 deg -> OK
PLAN OK
  PLAN OK
== Object 1: grasp, lift, put back ==
-> pre-grasp: tool xyz [0.283 0.225 0.138], theta [-180.     0.  -149.7]
   reached, position error 1.1 mm
   local table height differs from the far-range plane by more than 20 mm: continuing with the close-range height -0.0514 m
   close-range object: 788 points, centroid (0.2657, 0.2312), 17.7 mm from the far-range centroid
[2.1] centroid = [0.2657 0.2312 0.0025], object height 58 mm above the table
[2.2] 787 normals; mean angle to the centroid direction = 84.3 deg
[2.3] no usable antipodal pair; grasping across the body near its middle at (0.265, 0.233), local body direction [-0.88 -0.47], width 72 mm
[3] PCA eigenvalues [3.00e-06 6.59e-04 9.67e-04], major axis [-0.995 -0.09  -0.051]; approach [ 0.  0. -1.] via vertical (grasping across the body)
   close-range grasp (across_body): grasp point (0.2653, 0.2326) (far-range grasp point was (0.2834, 0.2254))
   close-range re-synthesis: approach tilt +0 deg, reachable
   refined grasp: approach tilt +0 deg (re-searched with IK), wrist rotation 88 deg
   refined: table z -0.0514, fingertip ends will stop at z -0.0281 (23.3 mm above the table)
-> pre-grasp (refined): tool xyz [0.265 0.233 0.107], theta [180.    0.  -62.1]
   reached, position error 0.8 mm
-> descend (stage 1): tool xyz [0.265 0.233 0.047], theta [180.    0.  -62.1]
   reached, position error 0.9 mm
-> descend to grasp (slow): tool xyz [0.265 0.233 0.007], theta [180.    0.  -62.1]
   reached, position error 0.6 mm
   gripper closed to 0.52, contact=True
-> lift: tool xyz [0.265 0.233 0.127], theta [180.    0.  -62.1]
   reached, position error 1.0 mm
   after lift gripper at 0.53 (was 0.52) -> HOLDING
-> lower to the table (place back): tool xyz [0.265 0.233 0.01 ], theta [180.    0.  -62.1]
   reached, position error 0.6 mm
   gripper opened smoothly -> 0.00
-> retreat up: tool xyz [0.265 0.233 0.127], theta [180.    0.  -62.1]
   reached, position error 1.0 mm
PICK + LIFT + RELEASE COMPLETE (place_back)
-> Object 1: PICK + LIFT + RELEASE COMPLETE (place_back)
== Object 2: perception ==
[1.1] scans: ['scan_0.npz', 'scan_1.npz', 'scan_2.npz', 'scan_3.npz', 'scan_4.npz', 'scan_5.npz', 'scan_6.npz', 'scan_7.npz', 'scan_8.npz', 'scan_9.npz']
[1.1] table of view 0 tilted 7.29 deg, offset +24.7 mm vs the centre view: aligned
[1.1] table of view 1 tilted 3.57 deg, offset +37.8 mm vs the centre view: aligned
[1.1] table of view 2 tilted 6.72 deg, offset +19.2 mm vs the centre view: aligned
[1.1] table of view 3 tilted 2.50 deg, offset +8.2 mm vs the centre view: aligned
[1.1] table of view 4 tilted 1.43 deg, offset +38.4 mm vs the centre view: aligned
[1.1] table of view 6 tilted 1.42 deg, offset -3.1 mm vs the centre view: aligned
[1.1] table of view 7 tilted 4.09 deg, offset +11.5 mm vs the centre view: aligned
[1.1] table of view 8 tilted 2.68 deg, offset +2.8 mm vs the centre view: aligned
[1.1] table of view 9 tilted 3.12 deg, offset +6.0 mm vs the centre view: aligned
[1.1] ICP scan 0 -> scan 5: shift 2.8 mm, rot 0.61 deg, trimmed rms 4.15 mm
[1.1] ICP scan 1 -> scan 5: shift 8.3 mm, rot 1.37 deg, trimmed rms 3.84 mm
[1.1] ICP scan 2 -> scan 5: shift 18.7 mm, rot 2.70 deg, trimmed rms 4.51 mm
[1.1] ICP scan 3 -> scan 5: shift 2.5 mm, rot 0.43 deg, trimmed rms 3.67 mm
[1.1] ICP scan 4 -> scan 5: shift 8.7 mm, rot 1.44 deg, trimmed rms 4.87 mm
[1.1] ICP scan 6 -> scan 5: shift 4.6 mm, rot 0.89 deg, trimmed rms 3.12 mm
[1.1] ICP scan 7 -> scan 5: shift 5.2 mm, rot 0.85 deg, trimmed rms 4.23 mm
[1.1] ICP scan 8 -> scan 5: shift 7.0 mm, rot 0.72 deg, trimmed rms 3.77 mm
[1.1] ICP scan 9 -> scan 5: shift 5.4 mm, rot 1.11 deg, trimmed rms 3.11 mm
[1.0] 10 viewpoints: obl_near_left (camera z 0.341 m), left_45 (camera z 0.345 m), obl_left (camera z 0.344 m), obl_near (camera z 0.339 m), left_22 (camera z 0.345 m), center (camera z 0.345 m), right_22 (camera z 0.345 m), obl_right (camera z 0.342 m), obl_near_right (camera z 0.340 m), right_45 (camera z 0.345 m)
[1.1] merged 928576 -> 213621 points (voxel 0.004 m)
[1.2] workspace: 112440 points
[1.3] plane normal=(-0.078,-0.034,0.996) -> 77226 inliers, z at x,y=0: -0.0556 m
[1.4] object points: 10955 (> 12 mm above plane; 5781 noise points and 2298 single-view points removed)
[1.5] 2 cluster(s):
      cluster 0: 6109 pts, extent [0.131 0.211 0.076], top 79 mm above table, centroid [0.283 0.227 0.015]
      cluster 1: 2241 pts, extent [0.113 0.109 0.061], top 70 mm above table, centroid [ 0.503 -0.075  0.011]
[2.1] centroids of all objects: object 0: (0.283, 0.227, 0.015); object 1: (0.503, -0.075, 0.011)
      target = object 1 of 2 on the table (nearest-first order)
[2.1] centroid = [ 0.5031 -0.075   0.0114], object height 56 mm above the table
[2.2] 2241 normals; mean angle to the centroid direction = 56.4 deg
[2.3] antipodal pair: p1=[ 0.5065 -0.0782  0.0139] p2=[ 0.4857 -0.086   0.0155] width=22.3 mm, mean grasp-point distance to the centroid=13.2 mm, cone quality=0.936
[3] PCA eigenvalues [0.000136 0.00041  0.000668], major axis [-0.644  0.74   0.194]; approach [-0.126  0.147 -0.981] via cross(PCA major axis, grasp axis)
  target at (0.503, -0.075) of 2 object(s)
== Object 2: grasp plan check (no motion) ==
pre_grasp: tool xyz [ 0.494 -0.049  0.134] theta [166.5   3.9  20.3] | bounds ok | IK max joint step 178 deg -> OK
grasp    : tool xyz [ 0.495 -0.074  0.037] theta [166.5   3.9  20.3] | bounds ok | IK max joint step 15 deg -> OK
lift     : tool xyz [ 0.495 -0.074  0.157] theta [166.5   3.9  20.3] | bounds ok | IK max joint step 20 deg -> OK
PLAN OK
  PLAN OK
== Object 2: grasp, lift, put back ==
-> pre-grasp: tool xyz [ 0.494 -0.049  0.134], theta [166.5   3.9  20.3]
   reached, position error 0.9 mm
   local table height differs from the far-range plane by more than 20 mm: continuing with the close-range height -0.0408 m
   close-range object: 415 points, centroid (0.4915, -0.0739), 11.7 mm from the far-range centroid
[2.1] centroid = [ 4.915e-01 -7.390e-02  4.000e-04], object height 53 mm above the table
[2.2] 415 normals; mean angle to the centroid direction = 54.8 deg
[2.3] no usable antipodal pair; grasping across the body near its middle at (0.491, -0.074), local body direction [-0.85  0.53], width 53 mm
[3] PCA eigenvalues [8.30e-05 2.50e-04 3.66e-04], major axis [ 0.641 -0.539  0.546]; approach [ 0.  0. -1.] via vertical (grasping across the body)
   close-range grasp (across_body): grasp point (0.4911, -0.0736) (far-range grasp point was (0.4961, -0.0821))
   close-range re-synthesis: approach tilt +0 deg, reachable
   refined grasp: approach tilt +0 deg (re-searched with IK), wrist rotation 40 deg
   refined: table z -0.0408, fingertip ends will stop at z -0.0195 (21.3 mm above the table)
-> pre-grasp (refined): tool xyz [ 0.491 -0.074  0.116], theta [180.   0.  58.]
   reached, position error 1.7 mm
-> descend (stage 1): tool xyz [ 0.491 -0.074  0.056], theta [180.   0.  58.]
   reached, position error 1.2 mm
-> descend to grasp (slow): tool xyz [ 0.491 -0.074  0.016], theta [180.   0.  58.]
   reached, position error 1.6 mm
   gripper closed to 0.87, contact=True
-> lift: tool xyz [ 0.491 -0.074  0.136], theta [180.   0.  58.]
   reached, position error 1.7 mm
   after lift gripper at 0.87 (was 0.87) -> HOLDING
-> lower to the table (place back): tool xyz [ 0.491 -0.074  0.019], theta [180.   0.  58.]
   reached, position error 1.3 mm
   gripper opened smoothly -> 0.00
-> retreat up: tool xyz [ 0.491 -0.074  0.136], theta [180.   0.  58.]
   reached, position error 1.6 mm
PICK + LIFT + RELEASE COMPLETE (place_back)
-> Object 2: PICK + LIFT + RELEASE COMPLETE (place_back)
== Object 3: perception ==
[1.1] scans: ['scan_0.npz', 'scan_1.npz', 'scan_2.npz', 'scan_3.npz', 'scan_4.npz', 'scan_5.npz', 'scan_6.npz', 'scan_7.npz', 'scan_8.npz', 'scan_9.npz']
[1.1] table of view 0 tilted 7.29 deg, offset +24.7 mm vs the centre view: aligned
[1.1] table of view 1 tilted 3.57 deg, offset +37.8 mm vs the centre view: aligned
[1.1] table of view 2 tilted 6.72 deg, offset +19.2 mm vs the centre view: aligned
[1.1] table of view 3 tilted 2.50 deg, offset +8.2 mm vs the centre view: aligned
[1.1] table of view 4 tilted 1.43 deg, offset +38.4 mm vs the centre view: aligned
[1.1] table of view 6 tilted 1.42 deg, offset -3.1 mm vs the centre view: aligned
[1.1] table of view 7 tilted 4.09 deg, offset +11.5 mm vs the centre view: aligned
[1.1] table of view 8 tilted 2.68 deg, offset +2.8 mm vs the centre view: aligned
[1.1] table of view 9 tilted 3.12 deg, offset +6.0 mm vs the centre view: aligned
[1.1] ICP scan 0 -> scan 5: shift 2.8 mm, rot 0.61 deg, trimmed rms 4.15 mm
[1.1] ICP scan 1 -> scan 5: shift 8.3 mm, rot 1.37 deg, trimmed rms 3.84 mm
[1.1] ICP scan 2 -> scan 5: shift 18.7 mm, rot 2.70 deg, trimmed rms 4.51 mm
[1.1] ICP scan 3 -> scan 5: shift 2.5 mm, rot 0.43 deg, trimmed rms 3.67 mm
[1.1] ICP scan 4 -> scan 5: shift 8.7 mm, rot 1.44 deg, trimmed rms 4.87 mm
[1.1] ICP scan 6 -> scan 5: shift 4.6 mm, rot 0.89 deg, trimmed rms 3.12 mm
[1.1] ICP scan 7 -> scan 5: shift 5.2 mm, rot 0.85 deg, trimmed rms 4.23 mm
[1.1] ICP scan 8 -> scan 5: shift 7.0 mm, rot 0.72 deg, trimmed rms 3.77 mm
[1.1] ICP scan 9 -> scan 5: shift 5.4 mm, rot 1.11 deg, trimmed rms 3.11 mm
[1.0] 10 viewpoints: obl_near_left (camera z 0.341 m), left_45 (camera z 0.345 m), obl_left (camera z 0.344 m), obl_near (camera z 0.339 m), left_22 (camera z 0.345 m), center (camera z 0.345 m), right_22 (camera z 0.345 m), obl_right (camera z 0.342 m), obl_near_right (camera z 0.340 m), right_45 (camera z 0.345 m)
[1.1] merged 928576 -> 213621 points (voxel 0.004 m)
[1.2] workspace: 112440 points
[1.3] plane normal=(-0.076,-0.032,0.997) -> 77556 inliers, z at x,y=0: -0.0553 m
[1.4] object points: 11492 (> 12 mm above plane; 6043 noise points and 2356 single-view points removed)
[1.5] 2 cluster(s):
      cluster 0: 6191 pts, extent [0.131 0.212 0.076], top 80 mm above table, centroid [0.283 0.228 0.014]
      cluster 1: 2314 pts, extent [0.113 0.116 0.062], top 71 mm above table, centroid [ 0.503 -0.075  0.011]
[2.1] centroids of all objects: object 0: (0.283, 0.228, 0.014); object 1: (0.503, -0.075, 0.011)
-> Object 3: NO_MORE_OBJECTS
  scan pose reached (joint-space move to the saved scan joints)
== Done: 2 object(s) picked and put back from one scan ==
```
