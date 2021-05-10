(define (domain waypoint_following)

    (:requirements
        :typing
        :durative-actions
        )

    (:types
        waypoint
        robot
    )
    
    (:predicates
        (visited ?wp - waypoint)
        (robot-at ?v - robot ?wp - waypoint)
    )
    
    (:durative-action move
        :parameters (?v - robot ?from ?to - waypoint)
        :duration ( = ?duration 2)
        :condition (and
            (at start (robot-at ?v ?from))
        )
        :effect (and
            (at end (visited ?to))
            (at end (robot-at ?v ?to))
            (at start(not
                (robot-at ?v ?from)
            ))
        )
    )    
)
