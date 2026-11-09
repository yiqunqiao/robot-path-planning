; Test 1: a single robot can reach its goal without collisions.
(define (problem p1-network)
    (:domain SingleRobotRunner)
    (:objects
        robot1 - robot
        square1 square2 square3 square4 square5 - gridsquare
        north east south west - direction
    )

    (:init
        ; Initial agent location and rotation
        (at-agent square1)
        (agent-rotated east)

        ; A straight five-square path
        (gridsquare-connected square1 square2 east)
        (gridsquare-connected square2 square1 west)
        (gridsquare-connected square2 square3 east)
        (gridsquare-connected square3 square2 west)
        (gridsquare-connected square3 square4 east)
        (gridsquare-connected square4 square3 west)
        (gridsquare-connected square4 square5 east)
        (gridsquare-connected square5 square4 west)

        ; Clockwise and anti-clockwise 90-degree rotations
        (rotation-adjacent north east)
        (rotation-adjacent east south)
        (rotation-adjacent south west)
        (rotation-adjacent west north)
        (rotation-adjacent east north)
        (rotation-adjacent south east)
        (rotation-adjacent west south)
        (rotation-adjacent north west)
    )

    (:goal (and
        (at-agent square5)
    ))
)
