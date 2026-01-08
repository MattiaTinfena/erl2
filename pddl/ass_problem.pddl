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
        (detecting robot1)
        (not(acquiring_imgs robot1))

        (= (num_id_detected) 0)
        (= (num_photo_taken) 0)

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

        (photo_untaken m1)
        (photo_untaken m2)
        (photo_untaken m3)
        (photo_untaken m4)

        (is_first m3)
        (is_next m4 m3)
        (is_next m1 m4)
        (is_next m2 m1)
    )

    (:goal
        ( and
            (photo_taken m1)
            (photo_taken m2)
            (photo_taken m3)
            (photo_taken m4)
        )
    )
)