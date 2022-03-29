import threading
try:
    import queue
except ImportError:
    import Queue as queue
import requests
import json
import traceback

URL = 'http://147.46.76.197:54242/api/put'

class ChConsumer(threading.Thread):
    """ Worker class that consumes URLs and generates thumbnails """
    def __init__(self, queue):
        self.queue = queue
        self.flag = True
        threading.Thread.__init__(self, name='consumer')

    def __str__(self):
        return 'Consumer'

    def stop(self):
        """ Stop the thread """
        self.flag = False

    def run(self):
        """ Main thread function """
        while self.flag:
            try:
                list_json = self.queue.get()
                if(list_json != 'exit'):
                    res = requests.post(URL, data=list_json)
            except:
                traceback.print_exc()