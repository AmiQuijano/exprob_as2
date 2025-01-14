(define (problem task)
(:domain domain_test)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    rob - robot
)
(:init
    (robot_at rob wp0)


    (unvisited wp0)
    (unvisited wp1)
    (unvisited wp2)
    (unvisited wp3)
    (unvisited wp4)

    (marker_detected wp0)

    (marker_undetected wp1)
    (marker_undetected wp2)
    (marker_undetected wp3)
    (marker_undetected wp4)


)
(:goal (and
    (arrived_lowest_waypoint)
))
)
