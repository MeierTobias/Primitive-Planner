def generate_launch_file(
    num_drones,
    init_x,
    init_y_spacing,
    goal_radius,
    output_file="decentralized_goal_flight.launch",
):
    with open(output_file, "w") as file:
        file.write("<launch>\n\n")

        # general env config
        file.write('    <arg name="map_size_x" value="500.0"/>\n')
        file.write('    <arg name="map_size_y" value="500.0"/>\n')
        file.write('    <arg name="map_size_z" value="3.0"/>\n')
        file.write('    <arg name="odom_topic" value="visual_slam/odom" />\n\n')

        # distributed communication
        file.write('    <arg name="sim_distributed_communication" value="true"/>\n')
        file.write('    <arg name="drone_com_r" value="15.0"/>\n')
        file.write('    <arg name="base_com_r" value="30.0"/>\n')
        file.write(f'    <arg name="goal_radius" value="{goal_radius}"/>\n')

        # swarm_bridge
        file.write("    <!-- swarm topic transmitter bridge-->\n")
        file.write(
            '    <include file="$(find swarm_bridge)/launch/bridge_udp.launch">\n'
        )
        file.write('        <arg name="drone_id" value="999"/>\n')
        file.write('        <arg name="broadcast_ip" value="127.0.0.255"/>\n')
        file.write("    </include>\n\n")

        # rviz setup
        file.write(
            '    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find primitive_planner)/launch/{}.rviz" required="true" />\n'.format(
                "verbose" if num_drones <= 40 else "drone_1000"
            )
        )
        file.write("\n")

        file.write(
            '    <node pkg="assign_goals" name="assign_goals_node" type="assign_goals_node" output="screen"/>\n\n'
        )

        # generate drone swarm
        init_y = [init_y_spacing * i for i in range(num_drones)]
        for i in range(num_drones):
            file.write("    <!-- Drone {} -->\n".format(i))
            file.write(
                '    <include file="$(find primitive_planner)/launch/run_in_sim.xml">\n'
            )
            file.write('        <arg name="drone_id" value="{}"/>\n'.format(i))
            file.write('        <arg name="init_x" value="{}"/>\n'.format(init_x))
            file.write('        <arg name="init_y" value="{}"/>\n'.format(init_y[i]))
            file.write('        <arg name="init_z" value="1.0"/>\n')
            file.write('        <arg name="flight_type" value="3"/>\n')
            file.write('        <arg name="map_size_x" value="$(arg map_size_x)"/>\n')
            file.write('        <arg name="map_size_y" value="$(arg map_size_y)"/>\n')
            file.write('        <arg name="map_size_z" value="$(arg map_size_z)"/>\n')
            file.write('        <arg name="odom_topic" value="$(arg odom_topic)"/>\n')
            file.write('        <arg name="goal_radius" value="$(arg goal_radius)"/>\n')
            file.write(
                '        <arg name="sim_distributed_communication" value="$(arg sim_distributed_communication)"/>\n'
            )
            file.write('        <arg name="drone_com_r" value="$(arg drone_com_r)"/>\n')
            file.write('        <arg name="base_com_r" value="$(arg base_com_r)"/>\n')
            file.write(
                '        <arg name="total_drones" value="{}"/>\n'.format(num_drones)
            )

            file.write("    </include>\n\n")

        file.write("</launch>")


if __name__ == "__main__":
    import sys
    import os

    if len(sys.argv) >= 2:
        drone_num = int(sys.argv[1])
    else:
        drone_num = 20
    launch_dir = (
        os.path.dirname(os.path.abspath(__file__)) + "/../planner/plan_manage/launch/"
    )

    init_x = -8.0
    init_y_spacing = 1.0
    goal_radius = 2.0  # Set your desired goal radius here

    generate_launch_file(
        drone_num,
        init_x,
        init_y_spacing,
        goal_radius,
        os.path.join(launch_dir, "decentralized_goal_flight.launch"),
    )
