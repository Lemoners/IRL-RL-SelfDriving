import subprocess
import os
import time

class MyTorcsProcess(object):

    def __init__(self):
        self.proc = None

    ## ------ Launch Torcs Env ----------- ##

    def start(self, sim_path, headless=False, port=None):

        # if not os.path.exists(sim_path):
            # print(sim_path, "does not exist. you must start sim manually.")
            # return

        # Launch Environment
        """
            Donnot know how to deal with headless now
        """
        try:
            if(self.proc!=None):
                self.quit()

            self.proc = subprocess.Popen(sim_path.split(" "))
                # print("proc",self.proc)
            # os.system(sim_path)
            # print("Torcs subprocess started")
            # time.sleep(3)
            # os.system("sh /home/lemon/Workspace/Torcs/MyTorcsEnv/torcs/mytorcs/test.sh")
        except Exception as e:
            print("Fail to excute Torcs subprocess: ",end="")
            print(e)

        

    def quit(self):
        """
        Shutdown unity environment
        """
        if self.proc is not None:
            # print("closing Torcs sim subprocess")

            self.proc.kill()
            # self.proc.wait()
            self.proc = None
        # os.system("pkill torcs")
    