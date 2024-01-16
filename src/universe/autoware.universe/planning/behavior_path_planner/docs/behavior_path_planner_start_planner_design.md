# Start Planner design

## Purpose / Role

The Start Planner module is designed to generate a path from the current ego position to the driving lane, avoiding static obstacles and implementing safety checks against dynamic obstacles. (Note: The feature of safety checks against dynamic obstacles is currently a work in progress.)
This module is activated when a new route is received.

Use cases are as follows

- start smoothly from the current ego position to centerline.
  ![case1](../image/start_from_road_lane.drawio.svg)
- pull out from the side of the road lane to centerline.
  ![case2](../image/start_from_road_side.drawio.svg)
- pull out from the shoulder lane to the road lane centerline.
  ![case3](../image/start_from_road_shoulder.drawio.svg)

## Design

```plantuml
@startuml
package start_planner{
    abstract class PullOutPlannerBase {
    }


    class ShiftPullOut {
    }

    class GeometricPullOut {
    }

    class StartPlannerModule {
    }

    struct PullOutPath{}
}

package utils{
    class PathShifter {
    }

    class GeometricParallelParking {
    }
}

' pull out
ShiftPullOut --|> PullOutPlannerBase
GeometricPullOut --|> PullOutPlannerBase

PathShifter --o ShiftPullOut
GeometricParallelParking --o GeometricPullOut

PullOutPlannerBase --o StartPlannerModule

PullOutPath --o PullOutPlannerBase

@enduml
```

## General parameters for start_planner

| Name                                                        | Unit  | Type   | Description                                                                 | Default value |
| :---------------------------------------------------------- | :---- | :----- | :-------------------------------------------------------------------------- | :------------ |
| th_arrived_distance_m                                       | [m]   | double | distance threshold for arrival of path termination                          | 1.0           |
| th_distance_to_middle_of_the_road                           | [m]   | double | distance threshold to determine if the vehicle is on the middle of the road |
| th_stopped_velocity_mps                                     | [m/s] | double | velocity threshold for arrival of path termination                          | 0.01          |
| th_stopped_time_sec                                         | [s]   | double | time threshold for arrival of path termination                              | 1.0           |
| th_turn_signal_on_lateral_offset                            | [m]   | double | lateral distance threshold for turning on blinker                           | 1.0           |
| intersection_search_length                                  | [m]   | double | check if intersections exist within this length                             | 30.0          |
| length_ratio_for_turn_signal_deactivation_near_intersection | [m]   | double | deactivate turn signal of this module near intersection                     | 0.5           |
| collision_check_margin                                      | [m]   | double | Obstacle collision check margin                                             | 1.0           |
| collision_check_distance_from_end                           | [m]   | double | collision check distance from end point. currently only for pull out        | 15.0          |
| center_line_path_interval                                   | [m]   | double | reference center line path point interval                                   | 1.0           |

## Safety check with static obstacles

1. Calculate ego-vehicle's footprint on pull out path between from current position to pull out end point. (Illustrated by blue frame)
2. Calculate object's polygon
3. If a distance between the footprint and the polygon is lower than the threshold (default: `1.0 m`), that is judged as a unsafe path

![pull_out_collision_check](../image/pull_out_collision_check.drawio.svg)

## Safety check with dynamic obstacles

WIP

## **Path Generation**

There are two path generation methods.

### **shift pull out**

This is the most basic method of starting path planning and is used on road lanes and shoulder lanes when there is no particular obstruction.

Pull out distance is calculated by the speed, lateral deviation, and the lateral jerk. The lateral jerk is searched for among the predetermined minimum and maximum values, and the one that generates a safe path is selected.

- Generate the road lane centerline and shift it to the current position.
- In the section between merge start and end, path is shifted by a method that is used to generate avoidance path (four segmental constant jerk polynomials)
- Combine this path with center line of road lane

![shift_pull_out](../image/shift_pull_out.drawio.svg)

[shift pull out video](https://user-images.githubusercontent.com/39142679/187872468-6d5057ee-e039-499b-afc7-fe0dc8052a6b.mp4)

#### parameters for shift pull out

| Name                            | Unit   | Type   | Description                                                                                                          | Default value |
| :------------------------------ | :----- | :----- | :------------------------------------------------------------------------------------------------------------------- | :------------ |
| enable_shift_pull_out           | [-]    | bool   | flag whether to enable shift pull out                                                                                | true          |
| check_shift_path_lane_departure | [-]    | bool   | flag whether to check if shift path footprints are out of lane                                                       | false         |
| shift_pull_out_velocity         | [m/s]  | double | velocity of shift pull out                                                                                           | 2.0           |
| pull_out_sampling_num           | [-]    | int    | Number of samplings in the minimum to maximum range of lateral_jerk                                                  | 4             |
| maximum_lateral_jerk            | [m/s3] | double | maximum lateral jerk                                                                                                 | 2.0           |
| minimum_lateral_jerk            | [m/s3] | double | minimum lateral jerk                                                                                                 | 0.1           |
| minimum_shift_pull_out_distance | [m]    | double | minimum shift pull out distance. if calculated pull out distance is shorter than this, use this for path generation. | 0.0           |
| maximum_curvature               | [m]    | double | maximum curvature. The pull out distance is calculated so that the curvature is smaller than this value.             |

| 0.07 |

### **geometric pull out**

Generate two arc paths with discontinuous curvature. Ego-vehicle stops once in the middle of the path to control the steer on the spot.
See also [[1]](https://www.sciencedirect.com/science/article/pii/S1474667015347431) for details of the algorithm.

![geometric_pull_out](../image/geometric_pull_out.drawio.svg)

[geometric pull out video](https://user-images.githubusercontent.com/39142679/181024707-3e7ca5ee-62de-4334-b9e9-ded313de1ea1.mp4)

#### parameters for geometric pull out

| Name                        | Unit  | Type   | Description                                                                                                                                                | Default value |
| :-------------------------- | :---- | :----- | :--------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| enable_geometric_pull_out   | [-]   | bool   | flag whether to enable geometric pull out                                                                                                                  | true          |
| divide_pull_out_path        | [-]   | bool   | flag whether to divide arc paths.　The path is assumed to be divided because the curvature is not continuous. But it requires a stop during the departure. | false         |
| geometric_pull_out_velocity | [m/s] | double | velocity of geometric pull out                                                                                                                             | 1.0           |
| arc_path_interval           | [m]   | double | path points interval of arc paths of geometric pull out                                                                                                    | 1.0           |
| lane_departure_margin       | [m]   | double | margin of deviation to lane right                                                                                                                          | 0.2           |
| pull_out_max_steer_angle    | [rad] | double | maximum steer angle for path generation                                                                                                                    | 0.26          |

## **backward pull out start point search**

If a safe path cannot be generated from the current position, search backwards for a pull out start point at regular intervals(default: `2.0`).

![pull_out_after_back](../image/pull_out_after_back.drawio.svg)

[pull out after backward driving video](https://user-images.githubusercontent.com/39142679/181025149-8fb9fb51-9b8f-45c4-af75-27572f4fba78.mp4)

### **parameters for backward pull out start point search**

| Name                          | Unit | Type   | Description                                                                                                                                                          | Default value  |
| :---------------------------- | :--- | :----- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------- |
| enable_back                   | [-]  | bool   | flag whether to search backward for start_point                                                                                                                      | true           |
| search_priority               | [-]  | string | In the case of `efficient_path`, use efficient paths even if the back distance is longer. In case of `short_back_distance`, use a path with as short a back distance | efficient_path |
| max_back_distance             | [m]  | double | maximum back distance                                                                                                                                                | 30.0           |
| backward_search_resolution    | [m]  | double | distance interval for searching backward pull out start point                                                                                                        | 2.0            |
| backward_path_update_duration | [s]  | double | time interval for searching backward pull out start point. this prevents chattering between back driving and pull_out                                                | 3.0            |
| ignore_distance_from_lane_end | [m]  | double | distance from end of pull out lanes for ignoring start candidates                                                                                                    | 15.0           |

### **freespace pull out**

If the vehicle gets stuck with pull out along lanes, execute freespace pull out.
To run this feature, you need to set `parking_lot` to the map, `activate_by_scenario` of [costmap_generator](../../costmap_generator/README.md) to `false` and `enable_freespace_planner` to `true`

<img src="https://user-images.githubusercontent.com/39142679/270964106-ae688bca-1709-4e06-98c4-90f671bb8246.png" width="600">

#### Unimplemented parts / limitations for freespace pull out

- When a short path is generated, the ego does can not drive with it.
- Complex cases take longer to generate or fail.
- The drivable area is not guaranteed to fit in the parking_lot.

#### Parameters freespace parking

| Name                           | Unit | Type   | Description                                                                                                                              | Default value |
| :----------------------------- | :--- | :----- | :--------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| enable_freespace_planner       | [-]  | bool   | this flag activates a free space pullout that is executed when a vehicle is stuck due to obstacles in the lanes where the ego is located | true          |
| end_pose_search_start_distance | [m]  | double | distance from ego to the start point of the search for the end point in the freespace_pull_out driving lane                              | 20.0          |
| end_pose_search_end_distance   | [m]  | double | distance from ego to the end point of the search for the end point in the freespace_pull_out driving lane                                | 30.0          |
| end_pose_search_interval       | [m]  | bool   | interval to search for the end point in the freespace_pull_out driving lane                                                              | 2.0           |

See [freespace_planner](../../freespace_planner/README.md) for other parameters.
