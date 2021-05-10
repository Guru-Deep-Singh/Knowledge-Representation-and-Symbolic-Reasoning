(define (problem waypoint_following)
    (:domain waypoint_following)
    (:requirements :strips :typing)

    (:objects   
                tiago - robot                                                   
                rightgrip - gripper
                wp0 wp_table_1 wp_table_2 wp_table_3 wp_cabinet_1 wp_cabinet_2 - waypoint
                aruco_cube_111 aruco_cube_222 aruco_cube_333 aruco_cube_444 aruco_cube_582 - object

    )
    
    (:init
                (visited wp0)                                                   ;Setting the starting waypoint as visited
                (robot-at tiago wp0)                                            ;Robot instance tiago starts from the middle of the room wp0
                (free rightgrip)                                                ;Initially the right gripper of the robot is free
                
                
                ;Costs definition for all the paths the robot can move in both directions
                ; The cost was calculated using the Euclidean distance formula and the xyz coordinates from the simulation yaml file
                (= (distance wp0 wp_table_1) 1.641463)                          
                (= (distance wp0 wp_table_2) 1.442221)
                (= (distance wp0 wp_table_3) 1.180381)
                (= (distance wp0 wp_cabinet_1) 1.329662)
                (= (distance wp0 wp_cabinet_2) 1.314286)
                
                (= (distance wp_table_1 wp_table_2) 1.92)
                (= (distance wp_table_1 wp_table_3) 2.608237)
                (= (distance wp_table_1 wp_cabinet_1) 2.917807)
                (= (distance wp_table_1 wp_cabinet_2)  2.534167)    
                (= (distance wp_table_1 wp0) 1.641463)
                
                (= (distance wp_table_2 wp0) 1.442221)
                (= (distance wp_table_2 wp_table_1) 1.92)
                (= (distance wp_table_2 wp_table_3) 1.287362)
                (= (distance wp_table_2 wp_cabinet_1) 2.460894)
                (= (distance wp_table_2 wp_cabinet_2) 2.735015)
                
                (= (distance wp_table_3 wp0) 1.180381)
                (= (distance wp_table_3 wp_table_1) 2.608237)
                (= (distance wp_table_3 wp_table_2) 1.287362)
                (= (distance wp_table_3 wp_cabinet_1) 1.397891)
                (= (distance wp_table_3 wp_cabinet_2) 2.01848)
                
                (= (distance wp_cabinet_1 wp0) 1.329662)
                (= (distance wp_cabinet_1 wp_table_1) 2.917807)
                (= (distance wp_cabinet_1 wp_table_2) 2.460894)
                (= (distance wp_cabinet_1 wp_table_3) 1.397891)
                (= (distance wp_cabinet_1 wp_cabinet_2) 0.9156)
                
                (= (distance wp_cabinet_2 wp0) 1.314286)
                (= (distance wp_cabinet_2 wp_table_1) 2.534167)
                (= (distance wp_cabinet_2 wp_table_2) 2.735015)
                (= (distance wp_cabinet_2 wp_table_3) 2.01848)
                (= (distance wp_cabinet_2 wp_cabinet_1)	0.9156)
                
                ;Defining the initial positions of the cube in the room
                (object-at aruco_cube_444 wp_table_3)
                (object-at aruco_cube_333 wp_table_3)
                (object-at aruco_cube_222 wp_table_2)
                (object-at aruco_cube_111 wp_table_2)
                (object-at aruco_cube_582 wp_table_1)
    )
    
    (:goal (and 
                    (object-at aruco_cube_444 wp_cabinet_2)     ;Object with aruco marker 444 (milk instance) needs to end up in cabinet 2
                    (object-at aruco_cube_222 wp_table_1)       ;Object with aruco marker 222 (yogurt instance) needs to end up in table 1
            )
    )
)

