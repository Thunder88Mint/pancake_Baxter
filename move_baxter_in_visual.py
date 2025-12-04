def move_baxter_in_visual(viz, qs, stepHoldTime, T_goal=None):
    if T_goal is not None:
        viz.add_frame(T_goal)
    
    for i in range(len(qs)):
        viz.update(qs=[qs[i]])
        viz.hold(stepHoldTime)

    if T_goal is not None:
        viz.remove_frame(-1)

    input('press Enter to see next iteration')