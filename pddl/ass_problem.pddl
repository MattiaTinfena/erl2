(define 
    (problem ass_problem)
    (:domain ass_domain)
    (:objects
        robot1 - robot
        base p1 p2 p3 p4 - point
        m1 m2 m3 m4 - marker
    )

    (:init
        (robot_at robot1 base)
        (robot_free robot1)

        (marker_at m1 p1)
        (marker_at m2 p2)
        (marker_at m3 p3)
        (marker_at m4 p4)

        (unvisited m1)
        (unvisited m2)
        (unvisited m3)
        (unvisited m4)

        (not (visited m1))
        (not (visited m2))
        (not (visited m3))
        (not (visited m4))

        (not (photo_taken m1))
        (not (photo_taken m2))
        (not (photo_taken m3))
        (not (photo_taken m4))

        ; (= (id m1) -1)
        ; (= (id m2) -1)
        ; (= (id m3) -1)
        ; (= (id m4) -1)

        (=(state robot1) 0)
    )

    (:goal
        ( and
            (visited m1)
            (visited m2)
            (visited m3)
            (visited m4)
            
            ;(=(state robot1) 1)
            ;(robot_at robot base)
        )
    )
)