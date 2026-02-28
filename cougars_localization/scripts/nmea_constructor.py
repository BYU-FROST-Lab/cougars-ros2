#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped  # For velocity
from nmea_msgs.msg import Sentence
import time
import serial
import math  # For atan2, sqrt, degrees


class NavSatToNmeaNode(Node):
    def __init__(self):
        super().__init__("nmea_converter")

        # Declare parameters
        serial_port_descriptor = ParameterDescriptor(
            description="The virtual serial port to write NMEA sentences to (e.g., /dev/tnt1)"
        )
        self.declare_parameter("serial_port", "/dev/tnt1", serial_port_descriptor)
        self.serial_port_name = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )

        self.declare_parameter(
            "baud_rate",
            9600,
            ParameterDescriptor(description="Baud rate for the serial port"),
        )
        self.baud_rate = (
            self.get_parameter("baud_rate").get_parameter_value().integer_value
        )

        self.declare_parameter(
            "nmea_talker_id",
            "GP",
            ParameterDescriptor(
                description="NMEA Talker ID (e.g., GP for GPS, GN for combined GNSS)"
            ),
        )
        self.nmea_talker_id = (
            self.get_parameter("nmea_talker_id").get_parameter_value().string_value
        )

        self.declare_parameter(
            "default_num_satellites",
            4,
            ParameterDescriptor(
                description="Default number of satellites if not updated otherwise (use >0 for fix)"
            ),
        )
        self.declare_parameter(
            "default_hdop",
            1.5,
            ParameterDescriptor(description="Default HDOP if not updated otherwise"),
        )
        self.declare_parameter(
            "default_geoid_separation",
            0.0,
            ParameterDescriptor(
                description="Default geoidal separation in meters if not updated otherwise"
            ),
        )

        # Subscribers
        self.navsat_subscription = self.create_subscription(
            NavSatFix,
            "gnss_1/llh_position",  # Source of position and fix status
            self.navsat_callback,
            10,
        )
        self.velocity_subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            "gnss_1/velocity",  # Source of speed and course
            self.velocity_callback,
            10,
        )

        self.publisher = self.create_publisher(Sentence, "nmea_constructed", 10)
        self.ser = None
        self._serial_warning_issued = False

        # To store the latest messages
        self.latest_navsat_fix = None
        self.latest_velocity = None

        try:
            self.ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
            self.get_logger().info(
                f"Successfully opened serial port: {self.serial_port_name} at {self.baud_rate} baud"
            )
        except serial.SerialException as e:
            self.get_logger().error(
                f"Could not open serial port {self.serial_port_name}: {e}"
            )
            self.get_logger().warn(
                "Proceeding without serial output. Will still publish to /nmea_constructed."
            )
            self.ser = None
            self._serial_warning_issued = True

        self.get_logger().info("NavSatFix to NMEA (and serial) converter node started")

    def _calculate_nmea_checksum(
        self, sentence_without_dollar_and_asterisk: str
    ) -> str:
        checksum = 0
        for char in sentence_without_dollar_and_asterisk:
            checksum ^= ord(char)
        return f"{checksum:02X}"

    def velocity_callback(self, msg: TwistWithCovarianceStamped):
        self.latest_velocity = msg
        # Optionally, if you want NavSatFix to be the primary trigger for sending combined data:
        # self._process_and_send_nmea()
        # Or, just store and let navsat_callback handle it. For simplicity, let navsat_callback trigger.

    def navsat_callback(self, msg: NavSatFix):
        self.latest_navsat_fix = msg
        self._process_and_send_nmea()

    def _process_and_send_nmea(self):
        if not self.latest_navsat_fix:
            return  # Need NavSatFix at least

        navsat_msg = self.latest_navsat_fix
        velocity_msg = self.latest_velocity  # Might be None if no velocity received yet

        # Construct GPGGA
        full_nmea_gga_string = self._construct_gpgga(navsat_msg)
        if full_nmea_gga_string:
            self._publish_and_send_serial(full_nmea_gga_string, navsat_msg.header.stamp)

        # Construct GPRMC
        full_nmea_rmc_string = self._construct_gprmc(navsat_msg, velocity_msg)
        if full_nmea_rmc_string:
            self._publish_and_send_serial(full_nmea_rmc_string, navsat_msg.header.stamp)

    def _publish_and_send_serial(self, nmea_string: str, stamp):
        nmea_sentence_msg = Sentence()
        nmea_sentence_msg.header.stamp = stamp
        nmea_sentence_msg.header.frame_id = "gps_link"
        nmea_sentence_msg.sentence = nmea_string
        self.publisher.publish(nmea_sentence_msg)

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(nmea_string.encode("ascii") + b"\r\n")
            except serial.SerialException as e:
                self.get_logger().error(
                    f"Error writing to serial port {self.serial_port_name}: {e}"
                )
            except Exception as e:
                self.get_logger().error(f"Unexpected error during serial write: {e}")
        elif not self._serial_warning_issued:
            self.get_logger().warn(
                f"Serial port {self.serial_port_name} not available. Skipping serial output.",
                throttle_duration_sec=30,
            )
            self._serial_warning_issued = True

    def _construct_gpgga(self, msg: NavSatFix) -> str:
        try:
            secs = msg.header.stamp.sec
            nanosecs = msg.header.stamp.nanosec
            t_utc = time.gmtime(secs)
            utc_time_str = f"{t_utc.tm_hour:02d}{t_utc.tm_min:02d}{t_utc.tm_sec:02d}.{nanosecs // 10000000:02d}"

            lat_abs = abs(msg.latitude)
            lat_deg = int(lat_abs)
            lat_min = (lat_abs - lat_deg) * 60.0
            lat_dir = "N" if msg.latitude >= 0 else "S"
            latitude_str = f"{lat_deg:02d}{lat_min:07.4f}"

            lon_abs = abs(msg.longitude)
            lon_deg = int(lon_abs)
            lon_min = (lon_abs - lon_deg) * 60.0
            lon_dir = "E" if msg.longitude >= 0 else "W"
            longitude_str = f"{lon_deg:03d}{lon_min:07.4f}"

            gps_qual = 0
            if msg.status.status == NavSatStatus.STATUS_NO_FIX:
                gps_qual = 0
            elif msg.status.status == NavSatStatus.STATUS_FIX:
                gps_qual = 1
            elif msg.status.status == NavSatStatus.STATUS_SBAS_FIX:
                gps_qual = 2
            elif msg.status.status == NavSatStatus.STATUS_GBAS_FIX:
                gps_qual = 2

            num_sats = (
                self.get_parameter("default_num_satellites")
                .get_parameter_value()
                .integer_value
            )
            num_sats_str = f"{num_sats:02d}"
            if num_sats == 0 and gps_qual > 0:
                self.get_logger().warn(
                    "GPGGA: Fix quality > 0 but num_sats is 0. GPSD might ignore this fix.",
                    throttle_duration_sec=10,
                )

            hdop = self.get_parameter("default_hdop").get_parameter_value().double_value

            geoid_sep = (
                self.get_parameter("default_geoid_separation")
                .get_parameter_value()
                .double_value
            )
            altitude_msl = msg.altitude - geoid_sep
            altitude_msl_str = f"{altitude_msl:.1f}"
            geoid_sep_str = f"{geoid_sep:.1f}"

            dgps_age_str = ""
            dgps_station_id_str = ""

            nmea_data_fields = [
                utc_time_str,
                latitude_str,
                lat_dir,
                longitude_str,
                lon_dir,
                f"{gps_qual}",
                num_sats_str,
                f"{hdop:.1f}",
                altitude_msl_str,
                "M",
                geoid_sep_str,
                "M",
                dgps_age_str,
                dgps_station_id_str,
            ]

            talker_id = (
                self.get_parameter("nmea_talker_id").get_parameter_value().string_value
            )
            sentence_id = talker_id + "GGA"
            sentence_core = sentence_id + "," + ",".join(nmea_data_fields)
            checksum_str = self._calculate_nmea_checksum(sentence_core)
            return f"${sentence_core}*{checksum_str}"
        except Exception as e:
            self.get_logger().error(f"Failed to construct GPGGA: {e}")
            return ""

    def _construct_gprmc(
        self, navsat_msg: NavSatFix, velocity_msg: TwistWithCovarianceStamped
    ) -> str:
        # $GPRMC,UTC,Status,Lat,N/S,Lon,E/W,Spd,Trak,Date,MagVar,MagVarDir,Mode*CS
        # Mode is NMEA 2.3, FAA mode for NMEA 4.1 (A=Autonomous, D=Differential, E=Estimated, N=Not valid, S=Simulator)
        if not navsat_msg:
            return ""
        try:
            # 1. UTC Time
            secs = navsat_msg.header.stamp.sec
            nanosecs = navsat_msg.header.stamp.nanosec
            t_utc = time.gmtime(secs)
            utc_time_str = f"{t_utc.tm_hour:02d}{t_utc.tm_min:02d}{t_utc.tm_sec:02d}.{nanosecs // 10000000:02d}"

            # 2. Status (A=Active/OK, V=Void/Warning)
            rmc_status = "V"
            mode_indicator = "N"  # NMEA 2.3+ mode indicator
            if navsat_msg.status.status == NavSatStatus.STATUS_NO_FIX:
                rmc_status = "V"
                mode_indicator = "N"
            elif navsat_msg.status.status == NavSatStatus.STATUS_FIX:
                rmc_status = "A"
                mode_indicator = "A"  # Autonomous
            elif (
                navsat_msg.status.status == NavSatStatus.STATUS_SBAS_FIX
                or navsat_msg.status.status == NavSatStatus.STATUS_GBAS_FIX
            ):
                rmc_status = "A"
                mode_indicator = "D"  # Differential

            # 3. Latitude, 4. N/S
            lat_abs = abs(navsat_msg.latitude)
            lat_deg = int(lat_abs)
            lat_min = (lat_abs - lat_deg) * 60.0
            lat_dir = "N" if navsat_msg.latitude >= 0 else "S"
            latitude_str = f"{lat_deg:02d}{lat_min:07.4f}"

            # 5. Longitude, 6. E/W
            lon_abs = abs(navsat_msg.longitude)
            lon_deg = int(lon_abs)
            lon_min = (lon_abs - lon_deg) * 60.0
            lon_dir = "E" if navsat_msg.longitude >= 0 else "W"
            longitude_str = f"{lon_deg:03d}{lon_min:07.4f}"

            # 7. Speed over ground (knots)
            # 8. Track angle (degrees True)
            speed_knots_str = ""
            track_deg_str = ""
            if (
                velocity_msg and rmc_status == "A"
            ):  # Only calculate if we have velocity and a fix
                # Assuming linear.x is East-velocity, linear.y is North-velocity (common for ENU frames)
                # If your /gnss_1/velocity frame_id is different or not ENU, this needs adjustment.
                vel_x = velocity_msg.twist.twist.linear.x  # East in m/s
                vel_y = velocity_msg.twist.twist.linear.y  # North in m/s
                speed_mps = math.sqrt(vel_x**2 + vel_y**2)
                speed_knots = speed_mps * 1.94384  # Conversion from m/s to knots
                speed_knots_str = f"{speed_knots:.1f}"

                if speed_knots > 0.1:  # Only calculate track if moving significantly
                    track_rad = math.atan2(vel_x, vel_y)  # Angle East of North
                    track_deg = math.degrees(track_rad)
                    if track_deg < 0:
                        track_deg += 360.0
                    track_deg_str = f"{track_deg:.1f}"
                else:  # Low speed, track is unreliable or undefined
                    track_deg_str = ""  # Or 0.0 if preferred for NMEA when speed is ~0

            # 9. Date (ddmmyy)
            date_str = f"{t_utc.tm_mday:02d}{t_utc.tm_mon:02d}{str(t_utc.tm_year)[-2:]}"

            # 10. Magnetic variation (degrees E/W) - often left null
            mag_var_str = ""
            # mag_var_dir_str = "" # If mag_var_str is empty, this is also typically empty or just one comma

            # NMEA 2.3 added a mode indicator field after mag_var_dir
            # $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,s.s,t.t,ddmmyy,m.m,a,m*CS
            #                                                             ^ mode

            nmea_data_fields = [
                utc_time_str,
                rmc_status,
                latitude_str,
                lat_dir,
                longitude_str,
                lon_dir,
                speed_knots_str,
                track_deg_str,
                date_str,
                mag_var_str,
                ""  # Mag Var and E/W dir; if mag_var_str is "", dir is also ""
                # For NMEA 2.3, add mode_indicator here
                # If using older NMEA spec that doesn't have mode, this field might not be present
                # Check what gpsd expects, often it's tolerant.
                # For simplicity, we'll add it.
                ,
                mode_indicator,
            ]

            # Filter out None from list in case any optional fields were conditional
            # However, for RMC, most fields up to date are expected.
            # The main optional fields are Speed, Track, MagVar.
            # NMEA requires commas for empty fields.

            talker_id = (
                self.get_parameter("nmea_talker_id").get_parameter_value().string_value
            )
            sentence_id = talker_id + "RMC"
            # Ensure all fields are strings before joining
            nmea_data_fields_str = [str(f) for f in nmea_data_fields]
            sentence_core = sentence_id + "," + ",".join(nmea_data_fields_str)
            checksum_str = self._calculate_nmea_checksum(sentence_core)
            return f"${sentence_core}*{checksum_str}"

        except Exception as e:
            self.get_logger().error(f"Failed to construct GPRMC: {e}")
            return ""

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.get_logger().info(f"Closing serial port: {self.serial_port_name}")
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavSatToNmeaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
