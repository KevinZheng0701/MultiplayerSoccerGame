from collections import deque


class ClientMessageQueue:
    """
    A per-client message queue that simulates network delay with in-order processing.
    Messages are added with a simulated arrival time. If messages arrive too soon after
    a previous delayed message, their processing is postponed to simulate a network pipeline effect.
    """

    def __init__(self):
        """
        Initialize the message queue and the earliest arrival tracker.
        """
        self.earliest_arrival = 0.0  # Time when the next message can be processed
        self.earliest_sent_time = -1  # Will store the first delayed send time
        self.queue = deque()  # Holds (arrival_time, send_time, message) details

    def add_message(self, arrival_time, sent_time, message):
        """
        Add a message to the queue with pipeline delay enforcement. If the queue already contains messages, this message's scheduled time is adjusted
        to not arrive earlier than the last message.

        Args:
            arrival_time (float): Simualted arrival time of the message.
            send_time (float): The original time the message was sent.
            message (str): The message object to store.
        """
        if self.queue:
            scheduled_time = max(arrival_time, self.queue[-1][0])
        else:
            scheduled_time = arrival_time
            self.earliest_arrival = arrival_time
            self.earliest_sent_time = sent_time

        self.queue.append((scheduled_time, sent_time, message))

    def is_queue_empty(self):
        """
        Check if the message queue is empty.

        Returns:
            bool: True if there are no messages in the queue, False otherwise.
        """
        return not self.queue

    def can_message_be_processed(self, curr_time):
        """
        Check if the message at the front of the queue can be processed.

        Args:
            curr_time (float): The current simulation time.

        Returns:
            bool: True if the first message is ready to be processed, False otherwise.
        """
        return not self.is_queue_empty() and curr_time >= self.earliest_arrival

    def get_top_message(self):
        """
        Retrieve and remove the first ready message from the queue.

        Returns:
            str: The message content, or "" if the queue is empty.
        """
        if not self.queue:
            return ""

        _, _, message = self.queue.popleft()
        if self.queue:
            self.earliest_arrival = self.queue[0][0]
            self.earliest_sent_time = self.queue[0][1]
        return message

    def get_earliest_arrival_time(self):
        """
        Get the earliest time at which the next message can be processed.

        Returns:
            float: The scheduled arrival time of the first message in the queue, or the last scheduled time if the queue is empty.
        """
        return self.earliest_arrival

    def get_earliest_sent_time(self):
        """
        Get the original send time of the first message in the queue.

        Returns:
            float: The send time of the earliest message, or the last sent time if the queue is empty.
        """
        return self.earliest_sent_time
