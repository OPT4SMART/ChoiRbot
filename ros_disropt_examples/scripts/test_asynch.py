from ros_disropt.communicators import ROSCommunicator
import time

def test_prepare(id, N, neigh, nreq=2):
    if N != nreq:
        raise Exception('This test requires N = {}'.format(nreq))

    communicator = ROSCommunicator(id, N, neigh, synchronous_mode=False)

    print('Agent {} starting'.format(id))
    time.sleep(4)

    return communicator

def test_asynch_basic(id, N, neigh):

    communicator = test_prepare(id, N, neigh)

    if id == 0:
        # send a message
        msg = 'message for 1'
        print('Sleeping 0.5s and sending a message...')
        time.sleep(0.5)
        communicator.neighbors_send(msg, neigh)
    else:
        # receive messages at t = 0, 1 s
        msgs = communicator.neighbors_receive_asynchronous(neigh)
        print('Received message at t = 0: ', msgs)
        time.sleep(1)
        msgs = communicator.neighbors_receive_asynchronous(neigh)
        print('Received message at t = 1: ', msgs)

    time.sleep(3)
    print('Agent {} exited'.format(id))

def test_asynch_multiple_msg(id, N, neigh):

    if id == 0:
        neigh = [1, 2]
        communicator = test_prepare(id, N, neigh, 3)

        # 2x (sleep 2s and receive)
        for i in range(2):
            print('Sleeping 2s and receiving messages (iter {})...'.format(i))
            time.sleep(2)
            msgs = communicator.neighbors_receive_asynchronous(neigh)
            print('Received messages: ', msgs)
    elif id == 1:
        neigh = [0]
        communicator = test_prepare(id, N, neigh, 3)
    
        # send, sleep 3s, sendx2
        print('Sending message...')
        communicator.neighbors_send('message no 1', neigh)
        print('Sleeping 3s...')
        time.sleep(3)
        print('Sending 2 messages...')
        communicator.neighbors_send('message no 2', neigh)
        communicator.neighbors_send('message no 3', neigh)
    elif id == 2:
        neigh = [0]
        communicator = test_prepare(id, N, neigh, 3)

        # send
        print('Sending message...')
        communicator.neighbors_send('msg no 1', neigh)
    else:
        communicator = test_prepare(id, N, neigh, 3)
    
    time.sleep(7)
    print('Agent {} exited'.format(id))
