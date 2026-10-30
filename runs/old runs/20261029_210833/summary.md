# Run 20261029_210833

**Result:** 1 object(s) picked and put back from one observation

```
  reached the scan pose via: joint-space move to the saved scan joints
[B] observation: predetermined viewpoints covering the table, one point cloud each (done once)
  view 0 obl_near_left: tool xyz [0.2606594074782259, 0.09278388238086374, 0.24619957228630873], oblique, looking in from 25 deg off vertical | largest joint step 59 deg
  view 1 left_45: tool xyz [0.20647518010647187, 0.13859292911256332, 0.259], swung +45 deg about the base | largest joint step 45 deg
  view 2 obl_left: tool xyz [0.3610489173914124, 0.13802229267049912, 0.2649715486559109], oblique, looking in from 25 deg off vertical | largest joint step 45 deg
  view 3 obl_near: tool xyz [0.22113748231039965, -0.006772142213574901, 0.2366012858056306], oblique, looking in from 25 deg off vertical | largest joint step 38 deg
  view 4 left_22: tool xyz [0.24379541068627827, 0.049028539936540126, 0.259], swung +22 deg about the base | largest joint step 23 deg
  view 5 center: tool xyz [0.244, -0.048, 0.259], swung +0 deg about the base | largest joint step 0 deg
  view 6 right_22: tool xyz [0.20705780117922964, -0.13772097505762365, 0.259], swung -22 deg about the base | largest joint step 22 deg
  view 7 obl_right: tool xyz [0.3608184944058099, -0.1508154797491188, 0.2590059875489409], oblique, looking in from 25 deg off vertical | largest joint step 57 deg
  view 8 obl_near_right: tool xyz [0.2611501532856921, -0.10584728922698719, 0.2420539974723641], oblique, looking in from 25 deg off vertical | largest joint step 37 deg
  view 9 right_45: tool xyz [0.13859292911256327, -0.20647518010647187, 0.259], swung -45 deg about the base | largest joint step 45 deg
view 0 (obl_near_left): 85182 points, camera at [0.281 0.126 0.341]
view 1 (left_45): 98136 points, camera at [0.245 0.18  0.345]
view 2 (obl_left): 83095 points, camera at [0.411 0.181 0.344]
view 3 (obl_near): 86966 points, camera at [ 0.228 -0.003  0.339]
view 4 (left_22): 98102 points, camera at [0.295 0.073 0.345]
view 5 (center): 96999 points, camera at [ 0.3   -0.046  0.345]
view 6 (right_22): 97674 points, camera at [ 0.26  -0.157  0.345]
view 7 (obl_right): 92141 points, camera at [ 0.411 -0.186  0.342]
view 8 (obl_near_right): 91914 points, camera at [ 0.281 -0.132  0.339]
view 9 (right_45): 96930 points, camera at [ 0.18  -0.244  0.345]
################ object 0 ################
[1.1] scans: ['scan_0.npz', 'scan_1.npz', 'scan_2.npz', 'scan_3.npz', 'scan_4.npz', 'scan_5.npz', 'scan_6.npz', 'scan_7.npz', 'scan_8.npz', 'scan_9.npz']
[1.1] table of view 0 tilted 5.24 deg, offset +22.7 mm vs the centre view: aligned
[1.1] table of view 1 tilted 2.20 deg, offset +33.7 mm vs the centre view: aligned
[1.1] table of view 2 tilted 6.01 deg, offset +18.7 mm vs the centre view: aligned
[1.1] table of view 3 tilted 1.16 deg, offset +6.8 mm vs the centre view: aligned
[1.1] table of view 4 tilted 0.86 deg, offset +31.8 mm vs the centre view: aligned
[1.1] table of view 6 tilted 2.05 deg, offset -4.8 mm vs the centre view: aligned
[1.1] table of view 7 tilted 5.65 deg, offset +11.1 mm vs the centre view: aligned
[1.1] table of view 8 tilted 4.22 deg, offset +2.7 mm vs the centre view: aligned
[1.1] table of view 9 tilted 3.26 deg, offset +4.1 mm vs the centre view: aligned
[1.1] ICP scan 0 -> scan 5: shift 5.1 mm, rot 0.69 deg, trimmed rms 3.85 mm
[1.1] ICP scan 1 -> scan 5: shift 6.3 mm, rot 0.99 deg, trimmed rms 3.13 mm
[1.1] ICP scan 2 -> scan 5: shift 12.9 mm, rot 2.07 deg, trimmed rms 3.96 mm
[1.1] ICP scan 3 -> scan 5: shift 3.5 mm, rot 0.41 deg, trimmed rms 3.30 mm
[1.1] ICP scan 4 -> scan 5: shift 6.2 mm, rot 0.93 deg, trimmed rms 4.18 mm
[1.1] ICP scan 6 -> scan 5: shift 7.9 mm, rot 1.41 deg, trimmed rms 2.77 mm
[1.1] ICP scan 7 -> scan 5: shift 3.4 mm, rot 0.43 deg, trimmed rms 3.78 mm
[1.1] ICP scan 8 -> scan 5: shift 3.9 mm, rot 0.57 deg, trimmed rms 3.43 mm
[1.1] ICP scan 9 -> scan 5: shift 6.2 mm, rot 1.25 deg, trimmed rms 2.76 mm
[1.0] 10 viewpoints: obl_near_left (camera z 0.341 m), left_45 (camera z 0.345 m), obl_left (camera z 0.344 m), obl_near (camera z 0.339 m), left_22 (camera z 0.345 m), center (camera z 0.345 m), right_22 (camera z 0.345 m), obl_right (camera z 0.342 m), obl_near_right (camera z 0.339 m), right_45 (camera z 0.345 m)
[1.1] merged 927139 -> 194241 points (voxel 0.004 m)
[1.2] workspace: 102737 points
[1.3] plane normal=(-0.061,-0.030,0.998) -> 78684 inliers, z at x,y=0: -0.0517 m
[1.4] object points: 8659 (> 12 mm above plane; 4391 noise points and 864 single-view points removed)
[1.5] 2 cluster(s):
      cluster 0: 3821 pts, extent [0.176 0.187 0.047], top 56 mm above table, centroid [0.435 0.073 0.003]
      cluster 1: 2687 pts, extent [0.136 0.103 0.099], top 105 mm above table, centroid [ 0.418 -0.333  0.021]
[2.1] centroids of all objects: object 0: (0.435, 0.073, 0.003); object 1: (0.418, -0.333, 0.021)
      target = object 0 of 2 on the table (nearest-first order)
[2.1] centroid = [0.4352 0.0732 0.0028], object height 45 mm above the table
[2.2] 3821 normals; mean angle to the centroid direction = 64.4 deg
[2.3] antipodal pair: p1=[0.4455 0.0765 0.0017] p2=[ 4.209e-01  6.530e-02 -3.000e-04] width=27.1 mm, mean grasp-point distance to the centroid=13.8 mm, cone quality=0.933
[3] PCA eigenvalues [9.500e-05 3.030e-04 2.935e-03], major axis [-0.622  0.783  0.017]; approach [ 0.054  0.065 -0.996] via cross(PCA major axis, grasp axis)
pre_grasp: tool xyz [0.426 0.062 0.13 ] theta [ 177.9    4.4 -155.6] | bounds ok | IK max joint step 73 deg -> OK
grasp    : tool xyz [0.431 0.069 0.03 ] theta [ 177.9    4.4 -155.6] | bounds ok | IK max joint step 16 deg -> OK
lift     : tool xyz [0.431 0.069 0.15 ] theta [ 177.9    4.4 -155.6] | bounds ok | IK max joint step 20 deg -> OK
PLAN OK
-> pre-grasp: tool xyz [0.426 0.062 0.13 ], theta [ 177.9    4.4 -155.6]
   reached, position error 1.3 mm
   close-range object: 705 points, centroid (0.4459, 0.0751), 10.9 mm from the far-range centroid
[2.1] centroid = [ 0.4459  0.0751 -0.0163], object height 32 mm above the table
[2.2] 705 normals; mean angle to the centroid direction = 71.7 deg
[2.3] no usable antipodal pair; grasping across the body near its middle at (0.450, 0.078), local body direction [-0.58  0.82], width 29 mm
[3] PCA eigenvalues [2.500e-05 1.190e-04 2.255e-03], major axis [-0.585  0.811  0.008]; approach [ 0.  0. -1.] via vertical (grasping across the body)
   close-range grasp (across_body): grasp point (0.4497, 0.0782) (far-range grasp point was (0.4332, 0.0709))
   close-range re-synthesis: approach tilt +0 deg, reachable
   refined grasp: approach tilt +0 deg (re-searched with IK), wrist rotation 12 deg
   refined: table z -0.0396, fingertip ends will stop at z -0.0266 (13.0 mm above the table)
-> pre-grasp (refined): tool xyz [0.45  0.078 0.108], theta [-180.     0.  -144.9]
   reached, position error 0.8 mm
-> descend (stage 1): tool xyz [0.45  0.078 0.048], theta [-180.     0.  -144.9]
   reached, position error 1.2 mm
-> descend to grasp (slow): tool xyz [0.45  0.078 0.008], theta [-180.     0.  -144.9]
   reached, position error 1.1 mm
   gripper closed to 1.00, contact=False
!! STOPPING: GraspAbort('gripper closed on nothing - not lifting')
--- object 0: SKIPPED: gripper closed on nothing - not lifting
################ object 1 ################
[1.1] scans: ['scan_0.npz', 'scan_1.npz', 'scan_2.npz', 'scan_3.npz', 'scan_4.npz', 'scan_5.npz', 'scan_6.npz', 'scan_7.npz', 'scan_8.npz', 'scan_9.npz']
[1.1] table of view 0 tilted 5.70 deg, offset +23.0 mm vs the centre view: aligned
[1.1] table of view 1 tilted 2.54 deg, offset +33.9 mm vs the centre view: aligned
[1.1] table of view 2 tilted 6.47 deg, offset +18.9 mm vs the centre view: aligned
[1.1] table of view 3 tilted 1.47 deg, offset +7.1 mm vs the centre view: aligned
[1.1] table of view 4 tilted 0.86 deg, offset +31.7 mm vs the centre view: aligned
[1.1] table of view 6 tilted 1.91 deg, offset -4.4 mm vs the centre view: aligned
[1.1] table of view 7 tilted 4.97 deg, offset +10.6 mm vs the centre view: aligned
[1.1] table of view 8 tilted 3.83 deg, offset +3.0 mm vs the centre view: aligned
[1.1] table of view 9 tilted 3.32 deg, offset +4.4 mm vs the centre view: aligned
[1.1] ICP scan 0 -> scan 5: shift 2.3 mm, rot 0.29 deg, trimmed rms 3.85 mm
[1.1] ICP scan 1 -> scan 5: shift 3.7 mm, rot 0.62 deg, trimmed rms 3.13 mm
[1.1] ICP scan 2 -> scan 5: shift 16.0 mm, rot 2.55 deg, trimmed rms 3.96 mm
[1.1] ICP scan 3 -> scan 5: shift 2.7 mm, rot 0.10 deg, trimmed rms 3.29 mm
[1.1] ICP scan 4 -> scan 5: shift 3.7 mm, rot 0.63 deg, trimmed rms 4.17 mm
[1.1] ICP scan 6 -> scan 5: shift 4.7 mm, rot 0.86 deg, trimmed rms 2.78 mm
[1.1] ICP scan 7 -> scan 5: shift 2.0 mm, rot 0.26 deg, trimmed rms 3.78 mm
[1.1] ICP scan 8 -> scan 5: shift 0.7 mm, rot 0.07 deg, trimmed rms 3.45 mm
[1.1] ICP scan 9 -> scan 5: shift 3.5 mm, rot 0.82 deg, trimmed rms 2.77 mm
[1.0] 10 viewpoints: obl_near_left (camera z 0.341 m), left_45 (camera z 0.345 m), obl_left (camera z 0.344 m), obl_near (camera z 0.339 m), left_22 (camera z 0.345 m), center (camera z 0.345 m), right_22 (camera z 0.345 m), obl_right (camera z 0.342 m), obl_near_right (camera z 0.339 m), right_45 (camera z 0.345 m)
[1.1] merged 927139 -> 194602 points (voxel 0.004 m)
[1.2] workspace: 102895 points
[1.3] plane normal=(-0.068,-0.029,0.997) -> 78770 inliers, z at x,y=0: -0.0539 m
[1.4] object points: 8241 (> 12 mm above plane; 3983 noise points and 801 single-view points removed)
[1.5] 2 cluster(s):
      cluster 0: 3605 pts, extent [0.173 0.17  0.048], top 56 mm above table, centroid [0.433 0.076 0.003]
      cluster 1: 2581 pts, extent [0.137 0.091 0.099], top 104 mm above table, centroid [ 0.418 -0.331  0.021]
[2.1] centroids of all objects: object 0: (0.433, 0.076, 0.003); object 1: (0.418, -0.331, 0.021)
      target = object 1 of 2 on the table (nearest-first order)
[2.1] centroid = [ 0.4184 -0.3312  0.0207], object height 90 mm above the table
[2.2] 2581 normals; mean angle to the centroid direction = 62.1 deg
[2.3] antipodal pair: p1=[ 0.4181 -0.3398  0.0103] p2=[ 0.4047 -0.3243  0.0091] width=20.5 mm, mean grasp-point distance to the centroid=16.4 mm, cone quality=0.952
[3] PCA eigenvalues [0.000139 0.000708 0.00104 ], major axis [0.974 0.211 0.08 ]; approach [ 0.083 -0.006 -0.997] via cross(PCA major axis, grasp axis)
pre_grasp: tool xyz [ 0.386 -0.344  0.127] theta [168.6   3.3 130.8] | bounds ok | IK max joint step 57 deg -> OK
grasp    : tool xyz [ 0.405 -0.335  0.029] theta [168.6   3.3 130.8] | bounds ok | IK max joint step 14 deg -> OK
lift     : tool xyz [ 0.405 -0.335  0.149] theta [168.6   3.3 130.8] | bounds ok | IK max joint step 22 deg -> OK
PLAN OK
-> pre-grasp: tool xyz [ 0.386 -0.344  0.127], theta [168.6   3.3 130.8]
   reached, position error 1.8 mm
   close-range object: 382 points, centroid (0.4239, -0.3588), 28.2 mm from the far-range centroid
[2.1] centroid = [ 0.4239 -0.3588  0.0521], object height 98 mm above the table
[2.2] 382 normals; mean angle to the centroid direction = 85.2 deg
[2.3] no usable antipodal pair; grasping across the body near its middle at (0.421, -0.358), local body direction [-0.91 -0.41], width 53 mm
[3] PCA eigenvalues [2.00e-06 2.43e-04 5.74e-04], major axis [-0.995 -0.096 -0.042]; approach [ 0.  0. -1.] via vertical (grasping across the body)
   close-range grasp (across_body): grasp point (0.4209, -0.3575) (far-range grasp point was (0.4114, -0.3320))
   close-range re-synthesis: approach tilt +24 deg, reachable
   refined grasp: approach tilt +24 deg (re-searched with IK), wrist rotation 21 deg
   refined: table z -0.0444, fingertip ends will stop at z -0.0144 (30.0 mm above the table)
-> pre-grasp (refined): tool xyz [ 0.371 -0.38   0.109], theta [156.   -0.  114.1]
   reached, position error 1.6 mm
-> descend (stage 1): tool xyz [ 0.393 -0.37   0.054], theta [156.   -0.  114.1]
   reached, position error 1.8 mm
-> descend to grasp (slow): tool xyz [ 0.408 -0.363  0.018], theta [156.   -0.  114.1]
   reached, position error 1.7 mm
   gripper closed to 0.82, contact=True
-> lift: tool xyz [ 0.408 -0.363  0.138], theta [156.   -0.  114.1]
   reached, position error 2.1 mm
   after lift gripper at 0.81 (was 0.82) -> HOLDING
-> lower to the table (place back): tool xyz [ 0.408 -0.363  0.021], theta [156.   -0.  114.1]
   reached, position error 1.6 mm
   gripper opened smoothly -> 0.00
-> retreat up: tool xyz [ 0.408 -0.363  0.138], theta [156.   -0.  114.1]
   reached, position error 1.6 mm
PICK + LIFT + RELEASE COMPLETE (place_back)
--- object 1: PICK + LIFT + RELEASE COMPLETE (place_back)
################ object 2 ################
[1.1] scans: ['scan_0.npz', 'scan_1.npz', 'scan_2.npz', 'scan_3.npz', 'scan_4.npz', 'scan_5.npz', 'scan_6.npz', 'scan_7.npz', 'scan_8.npz', 'scan_9.npz']
[1.1] table of view 0 tilted 5.70 deg, offset +23.0 mm vs the centre view: aligned
[1.1] table of view 1 tilted 2.54 deg, offset +33.9 mm vs the centre view: aligned
[1.1] table of view 2 tilted 6.47 deg, offset +18.9 mm vs the centre view: aligned
[1.1] table of view 3 tilted 1.47 deg, offset +7.1 mm vs the centre view: aligned
[1.1] table of view 4 tilted 0.86 deg, offset +31.7 mm vs the centre view: aligned
[1.1] table of view 6 tilted 1.91 deg, offset -4.4 mm vs the centre view: aligned
[1.1] table of view 7 tilted 4.97 deg, offset +10.6 mm vs the centre view: aligned
[1.1] table of view 8 tilted 3.83 deg, offset +3.0 mm vs the centre view: aligned
[1.1] table of view 9 tilted 3.32 deg, offset +4.4 mm vs the centre view: aligned
[1.1] ICP scan 0 -> scan 5: shift 2.3 mm, rot 0.29 deg, trimmed rms 3.85 mm
[1.1] ICP scan 1 -> scan 5: shift 3.7 mm, rot 0.62 deg, trimmed rms 3.13 mm
[1.1] ICP scan 2 -> scan 5: shift 16.0 mm, rot 2.55 deg, trimmed rms 3.96 mm
[1.1] ICP scan 3 -> scan 5: shift 2.7 mm, rot 0.10 deg, trimmed rms 3.29 mm
[1.1] ICP scan 4 -> scan 5: shift 3.7 mm, rot 0.63 deg, trimmed rms 4.17 mm
[1.1] ICP scan 6 -> scan 5: shift 4.7 mm, rot 0.86 deg, trimmed rms 2.78 mm
[1.1] ICP scan 7 -> scan 5: shift 2.0 mm, rot 0.26 deg, trimmed rms 3.78 mm
[1.1] ICP scan 8 -> scan 5: shift 0.7 mm, rot 0.07 deg, trimmed rms 3.45 mm
[1.1] ICP scan 9 -> scan 5: shift 3.5 mm, rot 0.82 deg, trimmed rms 2.77 mm
[1.0] 10 viewpoints: obl_near_left (camera z 0.341 m), left_45 (camera z 0.345 m), obl_left (camera z 0.344 m), obl_near (camera z 0.339 m), left_22 (camera z 0.345 m), center (camera z 0.345 m), right_22 (camera z 0.345 m), obl_right (camera z 0.342 m), obl_near_right (camera z 0.339 m), right_45 (camera z 0.345 m)
[1.1] merged 927139 -> 194602 points (voxel 0.004 m)
[1.2] workspace: 102895 points
[1.3] plane normal=(-0.067,-0.029,0.997) -> 78845 inliers, z at x,y=0: -0.0535 m
[1.4] object points: 8243 (> 12 mm above plane; 3986 noise points and 798 single-view points removed)
[1.5] 2 cluster(s):
      cluster 0: 3605 pts, extent [0.173 0.173 0.047], top 56 mm above table, centroid [0.433 0.076 0.003]
      cluster 1: 2582 pts, extent [0.137 0.091 0.099], top 104 mm above table, centroid [ 0.418 -0.331  0.021]
[2.1] centroids of all objects: object 0: (0.433, 0.076, 0.003); object 1: (0.418, -0.331, 0.021)
--- object 2: NO_MORE_OBJECTS
  reached the scan pose via: joint-space move to the saved scan joints
```
