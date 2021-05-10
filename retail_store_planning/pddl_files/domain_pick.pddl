(define (domain waypoint_following)

    (:requirements
        :typing
        :durative-actions
        )

    (:types
        waypoint
        robot
        object
        gripper
    )
    
    (:predicates
        (visited ?wp - waypoint)
        (robot-at ?v - robot ?wp - waypoint)
        (object-at ?obj - object ?wp - waypoint)
        (is_holding ?g - gripper ?obj - object)
        (free ?g - gripper)
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

    (:durative-action pick
        :parameters (?v - robot ?wp - waypoint ?obj - object ?g - gripper)
        :duration (= ?duration 2)
        :condition (and 
        (at start (robot-at ?v ?wp)) ;;not sure if visited ?wp should be added
        (at start (object-at ?obj ?wp))
        (at start (free ?g))
        
        )
        :effect (and
        (at end(is_holding ?g ?obj))
        (at end (not( object-at ?obj ?wp)))
        (at end(not(free ?g)))

        )
    )
    
)
