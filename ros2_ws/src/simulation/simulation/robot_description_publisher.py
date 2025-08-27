import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory

def resolve_path(p: str) -> str:
    """Hỗ trợ package://<pkg>/<path> và đường dẫn thường."""
    p = (p or "").strip()
    if p.startswith("package://"):
        rest = p[len("package://"):]
        if "/" not in rest:
            raise ValueError(f'Đường dẫn package:// không hợp lệ: "{p}"')
        pkg, rel = rest.split("/", 1)
        base = get_package_share_directory(pkg)
        return os.path.join(base, rel)
    return p

class RobotDescriptionSetter(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')

        # Params
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('target_node', 'robot_state_publisher')

        raw_urdf_path = str(self.get_parameter('urdf_path').value)
        self.target_node = str(self.get_parameter('target_node').value).strip()

        try:
            self.urdf_path = resolve_path(raw_urdf_path)
        except Exception as e:
            self.get_logger().error(f'Lỗi resolve urdf_path="{raw_urdf_path}": {e}')
            raise SystemExit(1)

        if not self.urdf_path or not os.path.exists(self.urdf_path):
            self.get_logger().error(f'URDF path không tồn tại: "{self.urdf_path}" (từ "{raw_urdf_path}")')
            raise SystemExit(1)

        try:
            with open(self.urdf_path, 'r') as f:
                self.urdf_content = f.read()
        except Exception as e:
            self.get_logger().error(f'Không đọc được URDF: {e}')
            raise SystemExit(1)

        # Service clients
        self.set_srv_name = f'/{self.target_node}/set_parameters'
        self.get_srv_name = f'/{self.target_node}/get_parameters'

        self.cli_set = self.create_client(SetParameters, self.set_srv_name)
        self.cli_get = self.create_client(GetParameters, self.get_srv_name)

        self.get_logger().info(f'Chờ service {self.set_srv_name} ...')
        if not self.cli_set.wait_for_service(timeout_sec=15.0):
            self.get_logger().error('Service set_parameters không sẵn sàng. robot_state_publisher đã chạy chưa?')
            raise SystemExit(1)

        self.get_logger().info(f'Chờ service {self.get_srv_name} ...')
        if not self.cli_get.wait_for_service(timeout_sec=5.0):
            # Không critical (một số node vẫn có), nhưng cảnh báo
            self.get_logger().warn('Service get_parameters chưa sẵn sàng, sẽ thử set trước.')
        self._push_once()

    # ----- Steps -----
    def _push_once(self):
        req = SetParameters.Request()
        p = Parameter()
        p.name = 'robot_description'
        pv = ParameterValue()
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = self.urdf_content
        p.value = pv
        req.parameters = [p]

        fut = self.cli_set.call_async(req)
        fut.add_done_callback(self._on_set_done)

    def _on_set_done(self, fut):
        ok = False
        try:
            result = fut.result()
            ok = bool(result and all(r.successful for r in result.results))
            if ok:
                self.get_logger().info('Đã set robot_description thành công. Xác minh lại...')
                self._verify()
            else:
                self.get_logger().warn('Set robot_description thất bại (service trả về unsuccessful).')
                self._shutdown(1)
                return
        except Exception as e:
            self.get_logger().error(f'Lỗi gọi set_parameters: {e}')
            self._shutdown(1)
            return

    def _verify(self):
        # Thử đọc lại tham số để chắc chắn node đã nhận
        if not self.cli_get.service_is_ready():
            self.get_logger().warn('get_parameters chưa sẵn sàng; bỏ qua bước xác minh.')
            self._shutdown(0)
            return

        from rcl_interfaces.msg import ParameterName
        req = GetParameters.Request()
        req.names = [ParameterName(name='robot_description')]
        fut = self.cli_get.call_async(req)
        fut.add_done_callback(self._on_get_done)

    def _on_get_done(self, fut):
        try:
            res = fut.result()
            if not res or len(res.values) == 0:
                self.get_logger().warn('Không đọc lại được robot_description.')
                self._shutdown(1)
                return

            val = res.values[0]
            ok = (val.type == ParameterType.PARAMETER_STRING) and (len(val.string_value) > 0)
            if ok:
                snippet = val.string_value[:80].replace('\n', ' ')
                self.get_logger().info(f'Xác minh OK. robot_description length={len(val.string_value)} '
                                       f'preview="{snippet}..."')
                self.get_logger().info('TIP: Mở RViz → Fixed Frame = base (hoặc base_link) và kiểm tra /tf.')
                self._shutdown(0)
            else:
                self.get_logger().warn('Giá trị robot_description không hợp lệ sau khi set.')
                self._shutdown(1)
        except Exception as e:
            self.get_logger().error(f'Lỗi gọi get_parameters: {e}')
            self._shutdown(1)

    def _shutdown(self, code: int):
        try:
            self.destroy_node()
        finally:
            rclpy.shutdown()
        # dùng SystemExit để kết thúc tiến trình với mã code
        raise SystemExit(code)

def main():
    rclpy.init()
    node = RobotDescriptionSetter()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass  # mã thoát đã được xử lý trong _shutdown
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()