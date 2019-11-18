#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Handler for topics, msgs, publisher, and subscriber.
@todo:
------------------------------------------------------------- """


from std_msgs.msg import Int16MultiArray, Int16
import rospy


def subscriber_callback(data):
    """ Callback method for subscriber.
    :param data:
    :return: data
    """
    return data.data


class TopicHandler:
    """ ROS Topic Handler.
    """

    def __init__(self, name, data_type):
        """ Init. method
        :param name: name of the topic
        :param data_type (eg: Int16MultiArray)
        """
        self._name = name
        self._data_type = data_type


class SubscribingHandler(TopicHandler):
    """ Subscribing Handler.
    """

    def __init__(self, name, data_type, callback=subscriber_callback):
        """ Init. method
        :param name: name of the topic
        :param data_type: (eg: Int16MultiArray)
        :param callback: for data
        """
        super().__init__(name, data_type)
        self._subscriber = rospy.Subscriber(self._name, self._data_type, callback)


class PublishingHandler(TopicHandler):
    """ Publishing Handler.
    """

    def __init__(self, name, data_type, queue_size=10):
        """ Init. method
        :param name: name of the topic
        :param data_type (eg: Int16MultiArray)
        :param queue_size
        """
        super().__init__(name, data_type)
        self._publisher = rospy.Publisher(self._name, self._data_type, queue_size=queue_size)

    def publish(self, data, frequency=0, quiet=True):
        """ Publishes data to the topic
        :param data:
        :param frequency
        :param quiet
        """
        if frequency == 0:
            self._publish_once(data, quiet)
        else:
            while not rospy.is_shutdown():
                self._publish_once(data, quiet)
                rospy.Rate(frequency).sleep()

    def _publish_once(self, data, quiet=True):
        """ Publishes data to the topic
        :param data:
        :param quiet
        """
        pub_data = self._data_type()
        pub_data.data = data
        if not quiet:
            rospy.loginfo("Publishing {0} to Topic {1}.".format(pub_data, self._name))
        self._publisher.publish(pub_data)
