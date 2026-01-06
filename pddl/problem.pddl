(define (problem aruco_problem)
  (:domain aruco_mission)

  (:objects
    robot1 - robot
    wp1 wp2 wp3 wp4 - waypoint
    c0 c1 c2 c3 c4 - count
  )

  (:init
    (dummy)

    (ec c0)
    (pc c0)

    (unexplored wp1)
    (unexplored wp2)
    (unexplored wp3)
    (unexplored wp4)

    (next c0 c1)
    (next c1 c2)
    (next c2 c3)
    (next c3 c4)

    (count_max c4)
  )

  (:goal (and
    (explored wp1)
    (explored wp2)
    (explored wp3)
    (explored wp4)
    (pc c4)
  ))
)
