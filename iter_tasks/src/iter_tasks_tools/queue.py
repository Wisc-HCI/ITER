from abc import ABCMeta, abstractmethod


class AbstractQueue:
    __metaclass__ = ABCMeta

    @abstractmethod
    def next(self):
        pass

    @abstractmethod
    def has_next(self):
        pass


class Queue(AbstractQueue):

    def __init__(self, start_pose, end_pose, num_items, orientation):
        super(Queue,self).__init__()
        self._start_pose = start_pose
        self._end_pose = end_pose
        self._orientation = orientation
        self._num_items = num_items
        self._count = 0

    def next(self):
        if self._count >= 1:
            return None

        dx = self._end_pose['position']['x'] - self._start_pose['position']['x']
        dy = self._end_pose['position']['y'] - self._start_pose['position']['y']
        dz = self._end_pose['position']['z'] - self._start_pose['position']['z']

        x = dx * self._count + self._start_pose['position']['x']
        y = dy * self._count + self._start_pose['position']['y']
        z = dz * self._count + self._start_pose['position']['z']

        self._count += 1 / self._num_items

        return dt.pose(dt.position(x,y,z),copy.deepcopy(self._orientation))

    def has_next(self):
        return self._count >= 1


class QueueSet(AbstractQueue):

    def __init__(self, queues=[]):
        super(QueueSet,self).__init__()
        self._queues = queues

    def next(self):
        for q in self._queues:
            if q.has_next():
                return q.next()
        return None

    def has_next(self):
        for q in self._queues:
            if q.has_next():
                return True
        return False
