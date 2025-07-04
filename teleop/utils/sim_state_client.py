import time 
import json
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.client import Client
from utils.server_api import SERVICE_NAME, API_VERSION, API_ID_GET_SIM_STATE


class SimStateClient(Client):
    def __init__(self, enableLease: bool = False):
        super().__init__(SERVICE_NAME, enableLease)

    def Init(self):
        self._RegistApi(API_ID_GET_SIM_STATE, 0)
        self._SetApiVerson(API_VERSION)

    def GetSimState_client_call(self):
        c, d = self._Call(API_ID_GET_SIM_STATE, json.dumps(""))
        return c, d



if __name__ ==  "__main__":
    # initialize channel factory.
    ChannelFactoryInitialize(0)

    # create client
    client = SimStateClient()
    client.Init()
    client.SetTimeout(5.0)

    # get server version
    code, serverApiVersion = client.GetServerApiVersion()
    print("server api version:", serverApiVersion)

    # wait lease applied
    client.WaitLeaseApplied()
    print("lease applied")
    # test api
    while True:
        try:
            c, d = client.GetSimState_client_call()
            print("client get sim state ret:", c)
            print("sim state data:", d)
        except Exception as e:
            print("Error getting sim state:", e)
        time.sleep(1.0)


