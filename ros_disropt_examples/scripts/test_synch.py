from ros_disropt.communicators import ROSCommunicator
import numpy as np
import time

def test_prepare(id, N, neigh, nreq=2):
    if N != nreq:
        raise Exception('This test requires N = {}'.format(nreq))

    communicator = ROSCommunicator(id, N, neigh)

    print('Agent {} starting'.format(id))
    time.sleep(3)

    return communicator

def test_basic(id, N, neigh):

    communicator = test_prepare(id, N, neigh)

    if id == 0:
        # send a message
        msg = 'message for 1'
        print('Sending a message...')
        communicator.neighbors_send(msg, neigh)
    else:
        # receive message
        msgs = communicator.neighbors_receive(neigh)
        print('Received message: ', msgs)

    time.sleep(3)
    print('Agent {} exited'.format(id))

def test_future(id, N, neigh):

    if id == 0:
        neigh = [1, 2]
        communicator = test_prepare(id, N, neigh, 3)

        # receive and measure time
        start_time = time.time()
        print('Receiving messages...')
        msgs = communicator.neighbors_receive(neigh)
        stop_time = time.time()
        print('Received messages: ', msgs)
        print('Elapsed time: ', stop_time - start_time)
    elif id == 1:
        neigh = [0]
        communicator = test_prepare(id, N, neigh, 3)
    
        # send
        print('Sending message...')
        communicator.neighbors_send('msg from 1', neigh)
    elif id == 2:
        neigh = [0]
        communicator = test_prepare(id, N, neigh, 3)

        # sleep 2 s and send
        print('Sleeping for 2s...')
        time.sleep(2)
        print('Sending message...')
        communicator.neighbors_send('msg from 2', neigh)
    else:
        communicator = test_prepare(id, N, neigh, 3)
    
    time.sleep(3)
    print('Agent {} exited'.format(id))

def test_cbgroups(id, N, neigh):

    if id == 0:
        neigh = [1, 2]
        communicator = test_prepare(id, N, neigh, 3)

        # receive three times
        for i in range(3):
            print('Receiving messages (iter {})'.format(i))
            msgs = communicator.neighbors_receive(neigh)
            print('Received messages: ', msgs)
    elif id == 1:
        neigh = [0]
        communicator = test_prepare(id, N, neigh, 3)
    
        # send 3 messages
        print('Sending 3 messages...')
        for i in range(3):
            communicator.neighbors_send('msg {}'.format(i), neigh)
    elif id == 2:
        neigh = [0]
        communicator = test_prepare(id, N, neigh, 3)

        # sleep 2s and send 3 messages
        print('Sleeping for 2s...')
        time.sleep(2)
        print('Sending 3 messages...')
        for i in range(3):
            communicator.neighbors_send('msg {}'.format(i), neigh)
    else:
        communicator = test_prepare(id, N, neigh, 3)

    time.sleep(4)
    print('Agent {} exited'.format(id))