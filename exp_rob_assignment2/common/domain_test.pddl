(define (domain domain_test)
(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
    waypoint
    robot
)

(:predicates
    (robot_at ?r - robot ?wp - waypoint)
    (visited ?wp - waypoint)
    (unvisited ?wp - waypoint)
    (marker_detected ?wp - waypoint)
    (marker_undetected ?wp - waypoint)
    (arrived_lowest_waypoint)
)

;; Visit an unvisited waypoint
(:durative-action goto_waypoint
    :parameters (?r - robot ?from ?to - waypoint)
    :duration ( = ?duration 60)
    :condition (and
        (at start (robot_at ?r ?from))
        (at start (unvisited ?to)))
    :effect (and
        (at end (visited ?to))
        (at end (not(unvisited ?to)))
        (at end (robot_at ?r ?to))
        (at start (not (robot_at ?r ?from))))
)

;; Rotate in place while searching for an Aruco marker
(:durative-action rotate_and_detect
    :parameters (?r - robot ?wp - waypoint)
    :duration (= ?duration 60)
    :condition (and 
        (at start (robot_at ?r ?wp))
        (at start (marker_undetected ?wp)))
    :effect (and 
        (at end (marker_detected ?wp))
        (at end (not(marker_undetected ?wp))))
)

;; Go to the waypoint corresponding to the lowest marker ID 
;; (logic for determining the lowest wp is done by a separate script)
(:durative-action goto_lowest_marker_wp
    :parameters (?r - robot)
    :duration (= ?duration 60)
    :condition (and 
        (at start (forall(?wp - waypoint) (marker_detected ?wp))))
    :effect (and 
        (at end (arrived_lowest_waypoint)))
)

)