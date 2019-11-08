#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Collection of traffic scenarios where the ego vehicle (hero)
is making a right turn
"""

from __future__ import print_function

import sys

import py_trees

import carla
from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenarios.basic_scenario import *
from srunner.tools.scenario_helper import *

TURNING_RIGHT_SIGNALIZED_JUNCTION_SCENARIOS = [
    "SignalizedJunctionRightTurn"
]


class SignalizedJunctionRightTurn(BasicScenario):

    """
    Implementation class for Hero
    Vehicle turning right at signalized junction scenario,
    Traffic Scenario 09.
    """
    category = "SignalizedJunctionLeftTurn"

    def __init__(self, world, ego_vehicle, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80):
        """
        Setup all relevant parameters and create scenario
        """
        self._target_vel = 25
        self._brake_value = 0.5
        self._ego_distance = 40
        self._traffic_light = None
        self._other_actor_transforms = []
        # Timeout of scenario in seconds
        self.timeout = timeout
        super(SignalizedJunctionRightTurn, self).__init__("HeroActorTurningRightAtSignalizedJunction",
                                                          ego_vehicle,
                                                          config,
                                                          world,
                                                          debug_mode,
                                                          criteria_enable=criteria_enable)

        # other vehicle's traffic light
        for other_actor in self.other_actors:
            traffic_light_other = CarlaDataProvider.get_next_traffic_light(other_actor, False)
            if traffic_light_other is None:
                print("No traffic light for the given location of the other vehicle found")
                sys.exit(-1)
            else:
                traffic_light_other.set_state(carla.TrafficLightState.Green)
                traffic_light_other.set_green_time(self.timeout)

        self._traffic_light = CarlaDataProvider.get_next_traffic_light(self.ego_vehicle, False)
        if self._traffic_light is None:
            print("No traffic light for the given location of the ego vehicle found")
            sys.exit(-1)
        self._traffic_light.set_state(carla.TrafficLightState.Red)
        self._traffic_light.set_red_time(self.timeout)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        for other_actor in config.other_actors:
            self._other_actor_transforms.append(other_actor.transform)
            vehicle_transform = carla.Transform(
                    carla.Location(other_actor.transform.location.x,
                        other_actor.transform.location.y,
                        other_actor.transform.location.z - 500),
                    other_actor.transform.rotation)
            vehicle = CarlaActorPool.request_new_actor(other_actor.model,
                    vehicle_transform)
            self.other_actors.append(vehicle)

    def _create_behavior_helper(self, sequence):
        location_of_collision_dynamic = get_geometric_linear_intersection(self.ego_vehicle, self.other_actors[0])
        crossing_point_dynamic = get_crossing_point(self.other_actors[0])
        sync_arrival = SyncArrival(
            self.other_actors[0], self.ego_vehicle, location_of_collision_dynamic)
        sync_arrival1 = SyncArrival(
            self.other_actors[1], self.ego_vehicle, location_of_collision_dynamic)
        sync_arrival2 = SyncArrival(
            self.other_actors[2], self.ego_vehicle, location_of_collision_dynamic)
        sync_arrival3 = SyncArrival(
            self.other_actors[3], self.ego_vehicle, location_of_collision_dynamic)
        sync_arrival4 = SyncArrival(
            self.other_actors[4], self.ego_vehicle, location_of_collision_dynamic)
        sync_arrival5 = SyncArrival(
            self.other_actors[5], self.ego_vehicle, location_of_collision_dynamic)
        sync_arrival_stop = InTriggerDistanceToLocation(self.other_actors[0], crossing_point_dynamic, 5)
        sync_arrival_stop1 = InTriggerDistanceToLocation(self.other_actors[1], crossing_point_dynamic, 5)
        sync_arrival_stop2 = InTriggerDistanceToLocation(self.other_actors[2], crossing_point_dynamic, 5)
        sync_arrival_stop3 = InTriggerDistanceToLocation(self.other_actors[3], crossing_point_dynamic, 5)
        sync_arrival_stop4 = InTriggerDistanceToLocation(self.other_actors[4], crossing_point_dynamic, 5)
        sync_arrival_stop5 = InTriggerDistanceToLocation(self.other_actors[5], crossing_point_dynamic, 5)

        sync_arrival_parallel = py_trees.composites.Parallel(
            "Synchronize arrival times",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sync_arrival_parallel.add_child(sync_arrival)
        sync_arrival_parallel.add_child(sync_arrival1)
        sync_arrival_parallel.add_child(sync_arrival2)
        sync_arrival_parallel.add_child(sync_arrival3)
        sync_arrival_parallel.add_child(sync_arrival4)
        sync_arrival_parallel.add_child(sync_arrival5)
        sync_arrival_parallel.add_child(sync_arrival_stop)
        sync_arrival_parallel.add_child(sync_arrival_stop1)
        sync_arrival_parallel.add_child(sync_arrival_stop2)
        sync_arrival_parallel.add_child(sync_arrival_stop3)
        sync_arrival_parallel.add_child(sync_arrival_stop4)
        sync_arrival_parallel.add_child(sync_arrival_stop5)

        move_actor_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        target_waypoint = generate_target_waypoint(
            CarlaDataProvider.get_map().get_waypoint(self.other_actors[0].get_location()), 0)
        plan = []
        wp_choice = target_waypoint.next(1.0)
        while not wp_choice[0].is_intersection:
            target_waypoint = wp_choice[0]
            plan.append((target_waypoint, RoadOption.LANEFOLLOW))
            wp_choice = target_waypoint.next(1.0)
        move_actor = WaypointFollower(self.other_actors[0], self._target_vel, plan=plan)
        waypoint_follower_end = InTriggerDistanceToLocation(
            self.other_actors[0], plan[-1][0].transform.location, 10)
        move_actor_parallel.add_child(move_actor)
        move_actor_parallel.add_child(waypoint_follower_end)


        target_waypoint = generate_target_waypoint(
            CarlaDataProvider.get_map().get_waypoint(self.other_actors[1].get_location()), 0)
        plan = []
        wp_choice = target_waypoint.next(1.0)
        count = 40
        while count > 0:
            count = count - 1
            target_waypoint = wp_choice[0]
            plan.append((target_waypoint, RoadOption.LANEFOLLOW))
            wp_choice = target_waypoint.next(1.0)
        move_actor = WaypointFollower(self.other_actors[1], self._target_vel, plan=plan)
        waypoint_follower_end = InTriggerDistanceToLocation(
            self.other_actors[1], plan[-1][0].transform.location, 0)
        move_actor_parallel.add_child(move_actor)
        move_actor_parallel.add_child(waypoint_follower_end)

        target_waypoint = generate_target_waypoint(
            CarlaDataProvider.get_map().get_waypoint(self.other_actors[4].get_location()), 0)
        plan = []
        wp_choice = target_waypoint.next(1.0)
        count = 40
        while count > 0:
            count = count - 1
            target_waypoint = wp_choice[0]
            plan.append((target_waypoint, RoadOption.LANEFOLLOW))
            wp_choice = target_waypoint.next(1.0)
        move_actor = WaypointFollower(self.other_actors[4], self._target_vel, plan=plan)
        waypoint_follower_end = InTriggerDistanceToLocation(
            self.other_actors[4], plan[-1][0].transform.location, 10)
        move_actor_parallel.add_child(move_actor)
        move_actor_parallel.add_child(waypoint_follower_end)

        target_waypoint = generate_target_waypoint(
            CarlaDataProvider.get_map().get_waypoint(self.other_actors[5].get_location()), 0)
        plan = []
        wp_choice = target_waypoint.next(1.0)
        count = 40
        while count > 0:
            count = count - 1
            target_waypoint = wp_choice[0]
            plan.append((target_waypoint, RoadOption.LANEFOLLOW))
            wp_choice = target_waypoint.next(1.0)
        move_actor = WaypointFollower(self.other_actors[5], self._target_vel, plan=plan)
        waypoint_follower_end = InTriggerDistanceToLocation(
            self.other_actors[5], plan[-1][0].transform.location, 10)
        move_actor_parallel.add_child(move_actor)
        move_actor_parallel.add_child(waypoint_follower_end)

        # stop other actor
        stop = StopVehicle(self.other_actors[0], self._brake_value)
        stop1 = StopVehicle(self.other_actors[1], self._brake_value)
        stop4 = StopVehicle(self.other_actors[4], self._brake_value)
        stop5 = StopVehicle(self.other_actors[5], self._brake_value)
        # end condition
        end_condition = DriveDistance(self.ego_vehicle, self._ego_distance)

        # Behavior tree
        sequence.add_child(ActorTransformSetter(self.other_actors[0],
            self._other_actor_transforms[0]))
        sequence.add_child(ActorTransformSetter(self.other_actors[1],
            self._other_actor_transforms[1]))
        sequence.add_child(ActorTransformSetter(self.other_actors[2],
            self._other_actor_transforms[2]))
        sequence.add_child(ActorTransformSetter(self.other_actors[3],
            self._other_actor_transforms[3]))
        sequence.add_child(ActorTransformSetter(self.other_actors[4],
            self._other_actor_transforms[4]))
        sequence.add_child(ActorTransformSetter(self.other_actors[5],
            self._other_actor_transforms[5]))
        sequence.add_child(sync_arrival_parallel)
        sequence.add_child(move_actor_parallel)
        sequence.add_child(stop)
        sequence.add_child(stop1)
        sequence.add_child(stop4)
        sequence.add_child(stop5)
        sequence.add_child(end_condition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))
        sequence.add_child(ActorDestroy(self.other_actors[2]))
        sequence.add_child(ActorDestroy(self.other_actors[3]))
        sequence.add_child(ActorDestroy(self.other_actors[4]))
        sequence.add_child(ActorDestroy(self.other_actors[5]))

    def _create_behavior(self):
        """
        Hero vehicle is turning right in an urban area,
        at a signalized intersection, while other actor coming straight
        from left.The hero actor may turn right either before other actor
        passes intersection or later, without any collision.
        After 80 seconds, a timeout stops the scenario.
        """

        sequence = py_trees.composites.Sequence()
        self._create_behavior_helper(sequence)

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicle)
        criteria.append(collison_criteria)

        return criteria

    def __del__(self):
        self._traffic_light = None
        self.remove_all_actors()
