from multiprocessing import Process
import os
def info(title):
    print(title)
    print("module_name: {}".format(__name__))
    print("parent process: {}".format(os.getppid()))
    print("process id: {}".format(os.getpid()))

def f(name):
    info('function f')
    print("hello", name)

if __name__ == "__main__":
    info('main line')
    p = Process(target=f, args=('bob',))
    p.start()
    p.join()