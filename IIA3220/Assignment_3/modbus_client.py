"""
Module containing modbus clients
A device uses 20 holding registers and 2 coils.
"""

import pymodbus.client as ModbusClient
from pymodbus import FramerType

class ModbusTemplateClient:
    """
    Template Modbus Class
    """

    def __init__(self, com_params, device_nr, client_type: str = "Template Client", test = True):
        # Extract params
        self.comm = com_params[0]
        self.host = com_params[1]
        self.port = com_params[2]
        self.framer = com_params[3]
        self.test = test
        self.client_type = client_type
        self.max_data_length = 20
        self.update_device_nr(device_nr)

        self.update_ip(self.host)

    def _connect(self):
        """Connecting to modbus server"""
        try:
            self.client.connect()
            if not self.client.connected:
                raise ConnectionError
            if self.test:
                print(f"{self.client_type}: Connected to server")
        except ConnectionError as e:
            if self.test:
                print(f"{self.client_type}: Could not connect")
            raise e

    def update_ip(self, ip: str):
        """TODOC"""

        self.host = ip

        # Set up client
        self.client: ModbusClient.ModbusBaseSyncClient
        if self.comm == "tcp":
            self.client = ModbusClient.ModbusTcpClient(self.host, port = self.port, framer = self.framer)
        elif self.comm == "udp":
            raise NotImplementedError
        elif self.comm == "serial":  # pragma: no cover
            raise NotImplementedError
        else:  # pragma: no cover
            print(f"Unknown client {self.comm} selected")
            raise ValueError

        self._connect()

    def update_device_nr(self, device_nr: int):
        """TODOC"""

        if device_nr > 5:
            raise ValueError(f"Too many devices! Limit is 4, you added nr {device_nr}")
        self.device_nr = self.max_data_length * device_nr

    def run_client(self, data):
        """Run sync client."""
        raise NotImplementedError

    def close(self):
        """Clean close of client connection"""
        if self.test:
            print(f"{self.client_type}: Closing client")
        self.client.close()

class ModbusWriteClient(ModbusTemplateClient):
    """
    Client used to write to a modbus server
    """

    def __init__(
            self,
            com_params: tuple[str, str, str, FramerType] = ("tcp", "127.0.0.1", "5020", FramerType.SOCKET),
            device_nr: int = 0,
            test: bool = False
            ):
        super().__init__(com_params, device_nr, "Write Client", test)

    def _encode(self, data):
        """Encode string to int"""
        data = str(int.from_bytes(str(data).encode("utf-8"))) # Encode data

        # Split data into sendable chunks
        encoded_data = []
        if int(data) > 65535:
            while int(data) > 65535:
                encoded_data.append(int(data[:4]))
                data = data[4:]
        encoded_data.append(int(data))
        if self.test:
            print(f"{self.client_type}: Encoded data = ", encoded_data)
        return encoded_data

    def run_client(self, data):
        """Writing to register and updating coil"""

        if not isinstance(data, int):
            send_data = self._encode(data)
            encoded = True
        else:
            send_data = [data]
            encoded = False

        if self.test:
            print(f"{self.client_type}: Writing register data: {data}")
        if len(send_data) > self.max_data_length:
            raise ValueError(f"data is to long: length = {len(send_data)}")
        for i, data_part in enumerate(send_data):
            self.client.write_register(self.device_nr + i + 1, data_part, device_id = 1)
        self.client.write_register(self.device_nr + 0, len(send_data), device_id = 1)
        if encoded:
            self.client.write_coil(self.device_nr + 1, True, device_id = 1)
        else:
            self.client.write_coil(self.device_nr + 1, False, device_id = 1)
        self.client.write_coil(self.device_nr + 0, True, device_id = 1)
        if self.client.read_coils(self.device_nr + 0, count=1, device_id = 1).bits[0]:
            if self.client.read_holding_registers(self.device_nr + 1, count=1, device_id = 1).registers[0] == send_data[0]:
                if self.test:
                    print("Successfully Updated")
            else:
                self.client.write_coil(self.device_nr + 0, False, device_id = 1)
                self.client.write_coil(self.device_nr + 1, False, device_id = 1)
                print("Connection Error")
        else:
            print("Connection Error")

class ModbusReadClient(ModbusTemplateClient):
    """
    Client used to read from modbus server
    """

    def __init__(
            self,
            com_params: tuple[str, str, str, FramerType] = ("tcp", "127.0.0.1", "5020", FramerType.SOCKET),
            device_nr: int = 0,
            test: bool = False
            ):
        super().__init__(com_params, device_nr, "Read Client", test)

    def _decode(self, data):
        """Decode int to string"""
        for i, data_part in enumerate(data[:-1]):
            data[i] = str(data_part)
            while len(data[i]) < 4:
                data[i] = "0" + data[i]
        data[-1] = str(data[-1])
        decoded_data = None
        while decoded_data is None:
            try:
                decoded_data = int("".join(data)).to_bytes(length=100).decode("utf-8")
            except UnicodeDecodeError:
                decoded_data = None
                data[-1] = "0" + data[-1]
                if len(data[-1]) > 5:
                    decoded_data = False
        if self.test:
            print(f"{self.client_type}: Decoded data = ", decoded_data)
        return decoded_data

    def run_client(self, data = None, read_if_updated = True) -> any:
        """Checking coil if register is updated and reading from register if so"""

        encoded = self.client.read_coils(self.device_nr + 1, count=1, device_id = 1).bits[0]
        if read_if_updated:
            if self.test:
                print(f"{self.client_type}: Checking coil")
            coil_status = self.client.read_coils(self.device_nr + 0, count=1, device_id = 1).bits[0]
            if self.test:
                print(f"{self.client_type}: Has register been updated? [True/False]", coil_status)
                if encoded:
                    print(f"{self.client_type}: Data is encoded.")
        else:
            coil_status = True

        if coil_status:
            register_response = self.client.read_holding_registers(self.device_nr + 1, count=1, device_id = 1).registers[0]
            self.client.write_coil(self.device_nr + 0, False, device_id = 1) # Marking register as not updated
            if encoded:
                register_response = [register_response]
                length = self.client.read_holding_registers(self.device_nr + 0, count=1, device_id = 1).registers[0]
                for i in range(length - 1):
                    register_response.append(self.client.read_holding_registers(self.device_nr + i + 2, count=1, device_id = 1).registers[0])
                register_response = self._decode(register_response)
            if self.test:
                print(f"{self.client_type}: Data: ", register_response)

            return register_response
        return None

if __name__ == "__main__":
    WriteClient = ModbusWriteClient(device_nr=1, test = True)
    WriteClient.run_client("2025/11/18 19:48:48")
    ReadClient = ModbusReadClient(device_nr=1, test = True)
    ReadClient.run_client()
    WriteClient.close()
    ReadClient.close()
