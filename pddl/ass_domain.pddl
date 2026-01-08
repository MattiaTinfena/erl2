(define (domain ass_domain)
    (:requirements :strips :typing :fluents :durative-actions 
    :negative-preconditions :numeric-fluents :disjunctive-preconditions 
    :quantified-preconditions :equality :continuous-effects)
    
    
    ; (:requirements :strips :typing :fluents :durative-actions :adl)

    (:types
        robot
        point
        marker
    )

    (:predicates
        (robot_at ?r - robot ?p - point)
        (marker_at ?m - marker ?p - point)
        (is_next ?m1 ?m2 - marker)
        (is_first ?m - marker)
        (visited ?m - marker)
        (unvisited ?m - marker)
        (photo_taken ?m - marker)
        (photo_untaken ?m - marker)
        (robot_free ?r - robot)
    )

    (:functions
        (state ?s - robot)
        ;(id ?id - marker)
    )

    (:durative-action detect_id
        :parameters (?r - robot ?p1 ?p2 - point ?m - marker)
        :duration ( = ?duration 5)
        :condition (and
            (at start(= (state ?r) 0))
            (over all(marker_at ?m ?p1))
            (at start(unvisited ?m))
            (at start(robot_at ?r ?p2))
            (at start (robot_free ?r))
        )
        :effect (and
            (at start(not(robot_free ?r)))
            (at end(robot_free ?r))
            (at end(robot_at ?r ?p1))
            (at end(not(robot_at ?r ?p2)))
            (at end(not(unvisited ?m)))
            (at end(visited ?m))
        )
    )
    

    (:action change_to_photo_state
        :parameters (?r - robot)
        :precondition (and 
            ;(forall (?m - marker) (visited ?m))
            (=(state ?r) 0)
        )
        :effect(and
            (assign (state ?r) 1)
        )
    )

    
    (:durative-action capture_img
        :parameters (?r - robot ?p1 ?p2 - point ?m1 ?m2 - marker)
        :duration ( = ?duration 5)
        :condition (and
            (at start(or 
                (and (photo_untaken ?m1) (is_first ?m1) (visited ?m1))
                (and (photo_untaken ?m1) (is_next ?m1 ?m2) (photo_taken ?m2))))
            (at start(= (state ?r) 1))
            (over all(marker_at ?m1 ?p1))
            (at start(robot_at ?r ?p2))
        )
        :effect (and
            (at end(robot_at ?r ?p1))
            (at end(not (robot_at ?r ?p2)))
            (at end(photo_taken ?m1))
            (at end(not(photo_untaken ?m1)))    
        )
    )
)