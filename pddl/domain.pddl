(define (domain aruco_mission)
  (:requirements :strips :typing)

  (:types
    robot waypoint count
  )

  (:predicates
    (dummy)

    (explored ?w - waypoint)
    (unexplored ?w - waypoint)

    (ec ?c - count)
    (pc ?c - count)

    (next ?c1 - count ?c2 - count)

    (count_max ?c - count)
  )

  (:action explore
    :parameters (?r - robot ?w - waypoint ?c1 - count ?c2 - count)
    :precondition (and
      (dummy)
      (unexplored ?w)
      (ec ?c1)
      (next ?c1 ?c2)
    )
    :effect (and
      (explored ?w)
      (not (unexplored ?w))
      (not (ec ?c1))
      (ec ?c2)
    )
  )

  (:action take_picture_next
    :parameters (?r - robot ?c1 - count ?c2 - count ?cmax - count)
    :precondition (and
      (dummy)
      (ec ?cmax)
      (count_max ?cmax)
      (pc ?c1)
      (next ?c1 ?c2)
    )
    :effect (and
      (not (pc ?c1))
      (pc ?c2)
    )
  )
)
