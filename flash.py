import logging
import udsoncan
import isotp
import time
from udsoncan.connections import IsoTPSocketConnection
from udsoncan.client import Client
from udsoncan.exceptions import *
from udsoncan.services import *

udsoncan.setup_logging()
logging.root.setLevel(logging.INFO)

uds_config = udsoncan.configs.default_client_config.copy()

chunk_size=214

tp_addr = isotp.Address(isotp.AddressingMode.Normal_11bits, rxid=0x70F, txid=0x707)

sock = isotp.socket()
sock.set_opts(txpad=0)

conn = IsoTPSocketConnection('can0', tp_addr, tpsock=sock)

with Client(conn, request_timeout=2, config=uds_config) as client:
    client.ecu_reset(ECUReset.ResetType.hardReset)
    time.sleep(1)
    
    try:
        client.change_session(DiagnosticSessionControl.Session.programmingSession) 
        client.request_download(memory_location= udsoncan.MemoryLocation(0x8001800, 0x10000, address_format=32, memorysize_format=32))
        
        with open('../OpenCANBox/cmake-build-releasebl/opencanbox.bin', "rb") as f:
            cntr = 0x01
            data = f.read(chunk_size)
            while data:
                client.transfer_data(cntr,data)
                cntr=cntr+1
                data = f.read(chunk_size)
    except Exception as e:
        print('An exception occurred: {}'.format(e))

    client.request_transfer_exit()
    client.ecu_reset(ECUReset.ResetType.hardReset)