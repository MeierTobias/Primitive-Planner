def generate_teleop_heading_launch_file(
    num_drones, init_x, init_y_spacing, output_file="teleop_heading_flight.launch"
):
    with open(output_file, "w") as file:
        file.write("<launch>\n\n")

        # General environment configuration
        file.write('    <arg name="map_size_x" value="500.0"/>\n')
        file.write('    <arg name="map_size_y" value="500.0"/>\n')
        file.write('    <arg name="map_size_z" value="3.0"/>\n')
        file.write('    <arg name="odom_topic" value="visual_slam/odom" />\n\n')

        # Communication arguments
        file.write('    <arg name="sim_distributed_communication" value="true"/>\n')
        file.write('    <arg name="drone_com_r" value="15.0"/>\n')
        file.write('    <arg name="base_com_r" value="30.0"/>\n')

        # Include swarm bridge
        file.write("    <!-- swarm topic transmitter bridge -->\n")
        file.write(
            '    <include file="$(find swarm_bridge)/launch/bridge_udp.launch">\n'
        )
        file.write('        <arg name="drone_id" value="999"/>\n')
        file.write('        <arg name="broadcast_ip" value="127.0.0.255"/>\n')
        file.write("    </include>\n\n")

        # RViz configuration
        file.write(
            '    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find primitive_planner)/launch/{}.rviz" required="true" />\n'.format(
                "verbose" if num_drones <= 40 else "drone_1000"
            )
        )
        file.write("\n")

        # Assign goals node
        file.write(
            '    <node pkg="assign_goals" name="assign_goals_node" type="assign_goals_node" output="screen"/>\n\n'
        )

        # Keyboard-based teleoperation node
        file.write("    <!-- teleop_twist_keyboard node for keyboard control -->\n")
        file.write(
            '    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="teleop_twist_keyboard" output="screen"/>\n'
        )

        # Drone spawns
        drone_com_r = 15.0
        init_y = [drone_com_r * 1.2 + init_y_spacing * i for i in range(num_drones - 1)]
        init_y.insert(0, 0.0)

        for i in range(num_drones):
            file.write("    <!-- Drone {} -->\n".format(i))
            file.write(
                '    <include file="$(find primitive_planner)/launch/run_in_sim.xml">\n'
            )
            file.write('        <arg name="drone_id" value="{}"/>\n'.format(i))
            file.write('        <arg name="init_x" value="{}"/>\n'.format(init_x))
            file.write('        <arg name="init_y" value="{}"/>\n'.format(init_y[i]))
            file.write('        <arg name="init_z" value="1.0"/>\n')
            file.write('        <arg name="flight_type" value="4"/>\n')
            file.write('        <arg name="map_size_x" value="$(arg map_size_x)"/>\n')
            file.write('        <arg name="map_size_y" value="$(arg map_size_y)"/>\n')
            file.write('        <arg name="map_size_z" value="$(arg map_size_z)"/>\n')
            file.write('        <arg name="odom_topic" value="$(arg odom_topic)"/>\n')
            file.write(
                '        <arg name="sim_distributed_communication" value="$(arg sim_distributed_communication)"/>\n'
            )
            file.write('        <arg name="drone_com_r" value="$(arg drone_com_r)"/>\n')
            file.write('        <arg name="base_com_r" value="$(arg base_com_r)"/>\n')
            file.write("    </include>\n\n")

        file.write("</launch>\n")


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

    generate_teleop_heading_launch_file(
        drone_num,
        init_x,
        init_y_spacing,
        os.path.join(launch_dir, "teleop_heading.launch"),
    )
