import xsensdeviceapi as xda
import keyboard
import time

from mtwCallback import MtwCallback

class MtwManipulator(xda.XsCallback):
    def __init__(self, control, wireless_master_callback, desired_update_rate, desired_radio_channel):
        super().__init__()
        self.m_control = control
        self.m_wmc = wireless_master_callback
        self.m_dur = desired_update_rate
        self.m_drc = desired_radio_channel

    def find_closest_update_rate(self, supported_update_rates):
        if not supported_update_rates:
            return 0

        if len(supported_update_rates) == 1:
            return supported_update_rates[0]

        closest_update_rate = min(supported_update_rates, key=lambda x: abs(x - self.m_dur))
        return closest_update_rate


    def scanDevicesAndExtractData(self):
        print("Scanning ports...")

        detected_devices = xda.XsScanner_scanPorts()

        print("Finding wireless master...")
        wireless_master_port = next((port for port in detected_devices if port.deviceId().isWirelessMaster()), None)
        if wireless_master_port is None:
            raise RuntimeError("No wireless masters found")

        print(f"Wireless master found @ {wireless_master_port}")

        print("Opening port...")
        if not self.m_control.openPort(wireless_master_port.portName(), wireless_master_port.baudrate()):
            raise RuntimeError(f"Failed to open port {wireless_master_port}")

        print("Getting XsDevice instance for wireless master...")
        wireless_master_device = self.m_control.device(wireless_master_port.deviceId())
        if wireless_master_device is None:
            raise RuntimeError(f"Failed to construct XsDevice instance: {wireless_master_port}")

        print(f"XsDevice instance created @ {wireless_master_device}")

        print("Setting config mode...")
        if not wireless_master_device.gotoConfig():
            raise RuntimeError(f"Failed to goto config mode: {wireless_master_device}")

        print("Attaching callback handler...")
        wireless_master_device.addCallbackHandler(self.m_wmc)

        print("Getting the list of the supported update rates...")
        supportUpdateRates = xda.XsDevice.supportedUpdateRates(wireless_master_device, xda.XDI_None)

        print("Supported update rates: ", end="")
        for rate in supportUpdateRates:
            print(rate, end=" ")
        print()

        
        new_update_rate = self.find_closest_update_rate(supportUpdateRates)

        print(f"Setting update rate to {new_update_rate} Hz...")

        if not wireless_master_device.setUpdateRate(new_update_rate):
            raise RuntimeError(f"Failed to set update rate: {wireless_master_device}")

        print("Disabling radio channel if previously enabled...")

        if wireless_master_device.isRadioEnabled():
            if not wireless_master_device.disableRadio():
                raise RuntimeError(f"Failed to disable radio channel: {wireless_master_device}")

        print(f"Setting radio channel to {self.m_drc} and enabling radio...")
        if not wireless_master_device.enableRadio(self.m_drc):
            raise RuntimeError(f"Failed to set radio channel: {wireless_master_device}")

        print("Waiting for MTW to wirelessly connect...\n")

        wait_for_connections = True
        connected_mtw_count = len(self.m_wmc.getWirelessMTWs())
        while wait_for_connections:
            time.sleep(0.1)
            next_count = len(self.m_wmc.getWirelessMTWs())
            if next_count != connected_mtw_count:
                print(f"Number of connected MTWs: {next_count}. Press 'Y' to start measurement.")
                connected_mtw_count = next_count

            wait_for_connections = not keyboard.is_pressed('y')



        print("Starting measurement...")
        if not wireless_master_device.gotoMeasurement():
            raise RuntimeError(f"Failed to goto measurement mode: {wireless_master_device}")

        print("Getting XsDevice instances for all MTWs...")
        all_device_ids = self.m_control.deviceIds()
        mtw_device_ids = [device_id for device_id in all_device_ids if device_id.isMtw()]
        mtw_devices = []
        for device_id in mtw_device_ids:
            mtw_device = self.m_control.device(device_id)
            if mtw_device is not None:
                mtw_devices.append(mtw_device)
            else:
                raise RuntimeError("Failed to create an MTW XsDevice instance")

        print("Attaching callback handlers to MTWs...")
        mtw_callbacks = [MtwCallback(i, mtw_devices[i]) for i in range(len(mtw_devices))]
        for i in range(len(mtw_devices)):
            mtw_devices[i].addCallbackHandler(mtw_callbacks[i])

        print("Creating a log file...")
        logFileName = "logfile.mtb"
        if wireless_master_device.createLogFile(logFileName) != xda.XRV_OK:
            raise RuntimeError("Failed to create a log file. Aborting.")
        else:
            print("Created a log file: %s" % logFileName)

        print("Starting recording...")
        ready_to_record = False

        while not ready_to_record:
            ready_to_record = all([mtw_callbacks[i].dataAvailable() for i in range(len(mtw_callbacks))])
            if not ready_to_record:
                print("Waiting for data available...")
                time.sleep(0.5)
            #     optional, enable heading reset before recording data, make sure all sensors have aligned physically the same heading!!
            else:
                for i in range(5):
                    time.sleep(0.5)
                    print("Do heading reset before recording data, make sure all sensors have aligned physically the same heading!!")
                    all([mtw_devices[i].resetOrientation(xda.XRM_Alignment) for i in range(len(mtw_callbacks))])

        if not wireless_master_device.startRecording():
            raise RuntimeError("Failed to start recording. Aborting.")

        print("\nMain loop. Press any key to quit\n")
        print("Waiting for data available...")
        return mtw_callbacks