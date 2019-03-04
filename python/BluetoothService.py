#!/usr/bin/python
from bluetooth import *
import sys
import select
import time

def delay(ms):
    time.sleep(ms/1000)

# Create global list of sockets
# If a test program throws an exception and terminates, all handles to open sockets in the class will be lost
#   In this case, the only way to severe existing connections (and thus be able to make new ones) is to use this global list
#   To sever these connections after a program terminates without disconnecting, call severOldConnections()
# So make sure that whatever file you're actually running includes the statement 'from BluetoothService import *'
#   This will ensure that the shell can see the below method after an unexpected termination
globalSockets = []

#def severOldConnections():
#    for socket in globalSockets[:]:
#        socket.close()
#        globalSockets.remove(socket)        

class BluetoothService:

    def __init__(self, enablePrint=True):
        self._discoveredDevices = {} # Dictionary mapping addresses to names for discovered devices
        self._addedDevices = {} # Dictionary mapping addresses to names for manually added devices
        self._sockets = {} # Dictionary mapping addresses to sockets
        
    def discoverDevices(self):
        """
        Discover available bluetooth devices.

        It seems that if multiple devices are available with the same name, only one of them will be found.
        
        @return: A dictionary mapping addresses to names of the discovered devices
        @rtype: dict{str:str}
        """
        print("Discovering Devices ...")
        # Perform multiple discoveries since single scans seem to miss some devices
        #   and the duration argument of discover_devices doesn't seem to work
        # Note discover_devices has a lookup_name argument but it crashes when set to true
        devices = {}
        addresses = []
        for i in range(1):
            newAddresses = discover_devices(duration=20) # 8-second discovery
            for address in newAddresses:
                if address not in addresses:
                    addresses.append(address)
        # Look up names from addresses
        for address in addresses:
            name = ''
            try:
                name = lookup_name(address, timeout=30)
            except IOError:
                print('Error looking up name for address <' + str(address) + '>')
            devices[address] = name
        print("Found %d devices:" % len(devices))
        for (address, name) in devices.items():
            print("  <%s> at <%s>" % (name, address))
        self._discoveredDevices = devices
        return devices.copy()

    def getDevices(self, nameFilters = ('')):
        """
        Return the known devices (both discovered and manually added).
        Optionally, only return ones with names including the given terms.

        Will not perform a new discovery (see L{discoverDevices} for that).

        @param nameFilters: A list/tuple of terms which must be contained (case insensitive) in the returned device names.
        @type nameFilters: str or list/tuple or str

        @return: Dictionary mapping addresses to names
        @rtype: dict
        """
        res = {}
        if not isinstance(nameFilters, (list, tuple)):
            nameFilters = [nameFilters]
        # Add devices that include the desired terms
        allDevices = dict(list(self._discoveredDevices.items()) + list(self._addedDevices.items()))
        for address in allDevices.keys():
            name = allDevices[address]
            for nameFilter in nameFilters:
                if name is None or len(name) == 0:
                    if nameFilter in address:
                        res[address] = name    
                else:
                    if nameFilter.lower() in name.lower():
                        res[address] = name
            if len(nameFilters) == 0:
                res[address] = name
        return res

    def addDevice(self, address, name=''):
        """
        Add a device to the list of available devices (as if it was discovered).

        @param address: The address of the device, of the format 'XX:XX:XX:XX:XX:XX'
        @type address: str

        @param name: The name of the device (optional)
        @type name: str
        """
        if not isinstance(address, str) or not isinstance(name, str):
            return False
        # Try to get the name if none was provided
        if len(name) == 0:
            try:
                name = lookup_name(address)
            except IOError:
                print('Error looking up name for address <' + str(address) + '> (but will still add the address)')
        # Store the new device
        self._addedDevices[address] = name

    def clearDevices(self, added=True, discovered=True):
        """
        Clears the known devices.

        @param added: Whether or not to clear the manually added devices (default True)
        @type added: boolean

        @param discovered: Whether or not to clear the discovered devices (default True)
        @type discovered: boolean
        """
        if added:
            self._addedDevices = {}
        if discovered:
            self._discoveredDevices = {}

    def getAddress(self, name):
        """
        Try to get the device address from the name.  It must have already been added.

        If an address is given, will just return that address.

        @param address: The device name
        @type address: str

        @return: The address of the device, or an empty string if unknown
        @rtype: str
        """
        if not isinstance(name, str):
            return False
        if name.count(':') == 5 and len(name.strip()) == 5 + 6*2:
            return name
        #print('\tLooking up address of <%s>' % name)
        allDevices = self.getDevices()
        if name not in allDevices.values():
            print('\t<%s> has not been discovered or added' % name)
            return ''
        for address in allDevices:
            if allDevices[address] == name:
                return address
                
    def connect(self, device, port=1, enablePrint=True):
        """
        Connect to a device with the given name or address
        To use device name, the device must have already been discovered (see L{discoverDevices}).

        The device should already be added to the computer's list of known devices.
        The PIN for the device should have been stored on the computer when it was added.
        
        @param device: The name or address of the device to connect (case sensitive)
        @type device: str
        @param port: The port to use (optional)
        @type deviceName: int

        @return: Whether or not the connection was successful
        @rtype: boolean
        """
        if not isinstance(device, str):
            return False
        if enablePrint:
            print('Connecting to <%s>' % device)
        # If a name was given, try to get its address
        device = self.getAddress(device)
        if len(device) == 0:
            return False
        # Create socket to use for connection (RFCOMM is default argument)
        socket = BluetoothSocket()
        connected = False
        # Try multiple times in case user is slow in following Windows prompt to add device
        # also, will try to disconnect and reconnect if already connected (maybe, not sure if this works)
        for count in range(4):
            if enablePrint:
                print('\tTrying to connect...')
            if enablePrint:
                print('\tIf prompted (on Windows), click to add device and enter PIN \'1234\'')
            try:
                socket.connect((device, port))
                connected = True
                break
            except IOError: # Got an error, probably either already connected or not been added to computer
                try:
                    socket.close()
                except IOError:
                    None
            socket = BluetoothSocket()
        # Check if connection was successful
        if connected:
            self._sockets[device] = socket
            globalSockets.append(socket)
            if enablePrint:
                print('\tConnected to ' + device + '!')
            return True
        else:
            print('\t*** Failed to connect to ' + device + ' ***')
            print('\tPossible solution (on Windows):')
            print('\t  -- Open Show Bluetooth Devices')
            print('\t  -- Click Add a Device')
            print('\t  -- Select the device ' + device)
            print('\t  -- Enter PIN 1234')
            print('\t  -- Rerun this program')                  
            return False

    def disconnect(self, device, enablePrint=True):
        """
        Disconect from the device with the given name or address.  Will do nothing if no connection has been made.

        @param device: The name or address of the device to disconnect (case sensitive).
        @type device: str

        @return: Whether device was disconnected (whether device was valid)
        @rtype: boolean
        """
        if not isinstance(device, str):
            return False
        if enablePrint:
            print('Disconnecting from <' + device + '>')
        # If a name was given, try to get its address
        device = self.getAddress(device)
        if len(device) == 0:
            return False
        # Disconnect device if a connection is stored
        if device not in self._sockets:
            return False
        socket = self._sockets[device]
        socket.close()
        del self._sockets[device]
        if socket in globalSockets:
            globalSockets.remove(socket)
        return True
            
    def send(self, device, message, enablePrint=False):
        """
        Send data to the device with the given name or address.

        @param device: The name or address of the device (case sensitive)
        @type device: str

        @param message: The data to send.  If it doesn't already end in '\0', '\0' will be appended.
        @type message: str or something which can be cast to str

        @return: Whether the data was sent.
        @rtype: boolean
        """
        if not isinstance(device, str):
            return False
        if enablePrint:
            print('Sending <' + message + '> to <' + device + '>')
        message = str(message)
        # If a name was given, try to get its address
        device = self.getAddress(device)
        if len(device) == 0:
            return False
        # Check that a connection has been made
        if device not in self._sockets:
            if not self.connect(device):
                print('\tHave not connected to <%s>' % device)
                return False
        # Check that message is null terminated (seems like this is done automatically in send but just in case)
        if message[-1] != '\0':
            message += '\0'
        # Send the message
        try:
            self._sockets[device].send(message)
            return True
        except IOError: # Stored socket is not actually open
            print('\tNot connected to <%s>' % device)
            del self._sockets[device]
            return False

    def read(self, device, timeout=1000, enablePrint=True):
        """
        Read data from the device with the given name or address.

        A connection must already have been made to the device (see L{connect}).

        @param device: The name or address of the device to poll (case sensitive)
        @type device: str

        @param timeout: The time to wait for a character before giving up, in milliseconds.  Default is 1000.
        @type timeout: int

        @return: The data that was received, or an empty string if timeout was reached or device was not connected.
        @rtpe: str
        """
        if not isinstance(device, str):
            return ''
        # If a name was given, try to get its address
        device = self.getAddress(device)
        if len(device) == 0:
            return ''
        # Check that a connection has been made
        if device not in self._sockets:
            print('\tHave not connected to <%s>' % device)
            return ''
        # Check if there is data available
        try:
            ready = select.select([self._sockets[device]], [], [], timeout/1000)[0]
        except select.error: # Socket is not actually open
            print('\tNot connected to <%s>' % device)
            del self._sockets[device]
            return ''
        # Read data until timeout is reached or data is terminated by null character
        # Due to bluetooth module timing, one call to 'read' may only get part of a string
        terminated = False
        data = ''
        timedout = False
        startTime = time.time()*1000
        while(ready and not terminated and not timedout):
            dataBytes = self._sockets[device].recv(1024)
            data += dataBytes.decode("utf-8")
            if '\0' in data:
                terminated = True
            if not terminated:
                ready = select.select([self._sockets[device]], [], [], 0.005 + timeout/1000)[0] # timeout arg for select is in seconds
            timedout = time.time()*1000 - startTime > timeout
        # Return data or empty string depending upon level of success
        if len(data) > 0 and terminated:
            data = data[0:-1]
            if enablePrint:
                print('Got data <' + data + '> from <' + device + '>')
            return data
        elif len(data) > 0 and not terminated:
            if enablePrint:
                print('Got data <' + data + '> from <' + device + '> but it did not terminate - be careful!')
            return data
        else:
            if enablePrint:
                print('Reached timeout trying to read from <' + device + '>')
            return ''

"""
Some test code...
"""
if __name__ == '__main__':
    bt = BluetoothService()
    bt.discoverDevices()
    
    devices = bt.getDevices()
    print('\nFound devices: ')
    for (name, address) in devices.items():
        print("  <%s> at <%s>" % (name, address))

    robots = bt.getDevices(('robot', 'hc-', 'linvor','20:13:12:02:18:92'))
    print('\nFound robot devices: ')
    toConnect = ''
    for (address, name) in robots.items():
        print("  <%s> at <%s>" % (name, address))
        toConnect = address

    ticker=0

    if bt.connect(toConnect):
        while(ticker < 11):
            bt.send(toConnect, "Ticker:" + str(ticker)+'\0')
            # bt.read(toConnect)
            delay(1000)
            # bt.send(toConnect, 'Hello World!')
            # bt.read(toConnect)
            ticker = ticker + 1
    delay(3000)
    bt.disconnect(toConnect)
   










