(define (problem waypoint_following)
    (:domain waypoint_following)
    (:requirements :strips :typing)

    (:objects   wp0 wp_table_1 wp_table_2 wp_table_3 - waypoint
                tiago - robot
    )
    (:init
        (visited wp0)
        (robot-at tiago wp0)
    )
    
    (:goal (and
        (visited wp_table_1)
	(visited wp_table_2)
	(visited wp_table_3)
        ))

)
