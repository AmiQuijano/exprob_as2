(define (problem problem_test)
(:domain domain_test)

(:objects rob - robot
wp0 wp1 wp2 wp3 wp4 - waypoint) ;; wp0 is the spawning position of the robot

(:init
    (robot_at rob wp0)
    (unvisited wp0)
    (unvisited wp1)
    (unvisited wp2)
    (unvisited wp3)
    (unvisited wp4)
    (marker_detected wp0) ;; Necessary to consider the spawn position as a waypoint even if it doesn't have a marker
    (marker_undetected wp1)
    (marker_undetected wp2)
    (marker_undetected wp3)
    (marker_undetected wp4)
)

;; The goal is to be at the waypoint corresponding to the marker with lowest ID
;; This happens after all waypoints have a detected marker
(:goal (and
    (arrived_lowest_waypoint))
)

)