(define (domain waypoint_following)

    (:requirements
        :typing
        :durative-actions
        :numeric-fluents
        )

    (:types
        waypoint
        robot
        object
        gripper
    )
    (:functions
                    (distance ?from - waypoint ?to - waypoint)                  ;Numeric fluent for optimizing the moving distance
     )
    (:predicates
                    (visited ?wp - waypoint)                                    ;Predicate to show the visited waypoints
                    (robot-at ?v - robot ?wp - waypoint)                        ;predicate to show at which waypoint the robot is
                    (object-at ?obj - object ?wp - waypoint)                    ;Predicate to show from which waypoint the object can be acessed
                    (is_holding ?g - gripper ?obj - object)                     ;Predicate to show that which gripper is holding which object
                    (free ?g - gripper)                                         ;Predicate to show that the gripper is free
    )
    
    (:durative-action move
        :parameters (?v - robot ?from ?to - waypoint)
        :duration (= ?duration (distance ?from ?to))                            ;Duration of the action move is based on the moving distance betweent the 2 waypoints
        
        :condition (and
                        (at start (robot-at ?v ?from))                          ;For a robot to be able to move it needs to be at the starting waypoint from which is moving from
                    )
        :effect (and
                        (at end (visited ?to))                                  ;At the end of the move action the robot to which the robot has moved is marked as visited
                        (at end (robot-at ?v ?to))                              ;At the end the robot is at the node in which it move to.
                        (at start(not (robot-at ?v ?from)))                     ;Once the robot has started moving then the robot is no longer at the starting waypoint.
                )
    )

    (:durative-action pick
        :parameters (?v - robot ?wp - waypoint ?obj - object ?g - gripper)
        :duration (= ?duration 2)                                               ;Pick action was assumed to be instanteneous thus constant duration
        :condition (and 
                        (over all (robot-at ?v ?wp))                            ; Through out picking an object the robot needs to be at a specific waypoint
                        (at start (object-at ?obj ?wp))                         ; The object needs to be at the same waypoint where the robot is
                        (at start (free ?g))                                    ; The gripper needs to be free at the begining of picking 
                    )
        :effect (and                                                            ; After performing the pick action
                        (at end(is_holding ?g ?obj))                            ; The object picked up is being hold by the gripper
                        (at end (not( object-at ?obj ?wp)))                     ; The object is no longer at a waypoint 
                        (at end(robot-at ?v ?wp))                               ; At the end of the action the robot is at the same waypoints
                        (at end (not(free ?g)))                                 ; At the end of the pick action the robot gripper is o longer free
                )
    )
    (:durative-action place
        :parameters (?v - robot ?wp - waypoint ?obj - object ?g - gripper)
        :duration (= ?duration 2)                                               ;Place action was assumed to be instanteneous thus constant duration
        :condition (and 
                        
                        (over all (robot-at ?v ?wp))                            ; Through out the placing action the robot needs to be at a specific waypoint
                        (at start (is_holding ?g ?obj))                         ; At the begining of the place action the gripper of the robot needs to be holding an object
                        
                   )
        :effect    (and                                                         ; After performing the action place
                        (at end (not(is_holding ?g ?obj) ) )                    ; The gripper is no longer holding an object
                        (at end (object-at ?obj ?wp ) )                         ; At the end of the action the object is at the waypoint in which it was placed
                        (at end(robot-at ?v ?wp))                               ; At the end of the place action the robobt is stil at the waypoint it was before
                        (at end (free ?g) )                                     ; At the end of the place action the gripper is free since it place the object.
                    )
    )
)

