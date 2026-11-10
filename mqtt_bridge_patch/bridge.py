
# -*- coding: utf-8 -*-
from __future__ import absolute_import

from abc import ABCMeta
import math
import threading

import inject
import paho.mqtt.client as mqtt
import rospy

from .util import lookup_object, extract_values, populate_instance

try:
    basestring
except NameError:
    basestring = str  # kompatibilitas ROS1 di Python 3 (Noetic)


# ================================================================
# PENGATURAN KECEPATAN (hanya untuk ROS geometry_msgs/Twist -> MQTT)
# Linear  : m/s; angular: rad/s
# Accel   : m/s^2; angular accel: rad/s^2
# Bisa ditimpa lewat ROS private params (~max_linear, dst.), atau
# lewat argumen konstruktor RosToMqttBridge.
# ================================================================
MAX_LINEAR = 0.30            # batas magnitudo kecepatan linear (m/s)
MAX_ANGULAR = 0.80           # batas magnitudo kecepatan angular (rad/s)
MAX_LINEAR_ACCEL = 0.40      # perubahan linear maksimum per detik (m/s^2)
MAX_ANGULAR_ACCEL = 1.20     # perubahan angular maksimum per detik (rad/s^2)
SMOOTHING_TAU = 0.15         # low-pass time constant (detik); 0 = tanpa filter
CMD_TIMEOUT = 0.50           # jika ROS berhenti mengirim: paksa output nol (detik)
DEFAULT_VELOCITY_HZ = 50.0   # frekuensi publish MQTT untuk Twist saat frequency=None


def _finite_float(value):
    """Nilai tidak valid/NaN/Inf diperlakukan sebagai 0 (fail safe)."""
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return 0.0
    if math.isnan(value) or math.isinf(value):
        return 0.0
    return value


def _norm3(values):
    """Norma tiga dimensi yang tahan terhadap float dengan magnitudo besar."""
    scale = max(abs(value) for value in values)
    if scale == 0.0:
        return 0.0
    return scale * math.sqrt(sum((value / scale) ** 2 for value in values))


def _clamp_vector(values, limit):
    """Batasi magnitudo vektor; arah tetap dipertahankan.

    Hitungan dinormalisasi untuk mencegah overflow walau input sangat besar.
    """
    scale = max(abs(value) for value in values)
    if scale == 0.0:
        return [0.0, 0.0, 0.0]
    unit = [value / scale for value in values]
    unit_norm = math.sqrt(sum(value * value for value in unit))
    if scale <= limit / unit_norm:
        return list(values)
    return [value * (limit / unit_norm) for value in unit]


def _slew_vector(previous, target, max_accel, dt):
    """Batasi besarnya perubahan vektor tiap update agar tidak melonjak."""
    difference = [target[i] - previous[i] for i in range(3)]
    magnitude = _norm3(difference)
    max_delta = max_accel * dt
    if magnitude <= max_delta:
        return list(target)
    if magnitude == 0.0 or max_delta <= 0.0:
        return list(previous)
    portion = max_delta / magnitude
    return [previous[i] + difference[i] * portion for i in range(3)]


def _read_vector(vector):
    return [_finite_float(vector.x), _finite_float(vector.y),
            _finite_float(vector.z)]


def _write_vector(vector, values):
    vector.x, vector.y, vector.z = values


def _config_float(explicit, param_name, default, allow_zero=False):
    value = rospy.get_param(param_name, default) if explicit is None else explicit
    value = float(value)
    if math.isnan(value) or math.isinf(value) or (value < 0 if allow_zero else value <= 0):
        raise ValueError('{} must be a finite {} number'.format(
            param_name, 'non-negative' if allow_zero else 'positive'))
    return value


def create_bridge(factory, msg_type, topic_from, topic_to, **kwargs):
    u"""Create a ROS/MQTT bridge instance."""
    if isinstance(factory, basestring):
        factory = lookup_object(factory)
    if not issubclass(factory, Bridge):
        raise ValueError("factory should be Bridge subclass")
    if isinstance(msg_type, basestring):
        msg_type = lookup_object(msg_type)
    if not issubclass(msg_type, rospy.Message):
        raise TypeError(
            "msg_type should be rospy.Message instance or its string"
            "reprensentation")
    return factory(
        topic_from=topic_from, topic_to=topic_to, msg_type=msg_type, **kwargs)


class Bridge(object):
    u"""Base class for bridges."""
    __metaclass__ = ABCMeta

    _mqtt_client = inject.attr(mqtt.Client)
    _serialize = inject.attr('serializer')
    _deserialize = inject.attr('deserializer')
    _extract_private_path = inject.attr('mqtt_private_path_extractor')


class RosToMqttBridge(Bridge):
    u"""Bridge ROS -> MQTT.

    geometry_msgs/Twist automatically uses a periodic, rate-limited output.
    Other ROS message types keep the original callback-driven behavior.
    """

    def __init__(self, topic_from, topic_to, msg_type, frequency=None,
                 max_linear=None, max_angular=None, max_linear_accel=None,
                 max_angular_accel=None, smoothing_tau=None, cmd_timeout=None):
        self._topic_from = topic_from
        self._topic_to = self._extract_private_path(topic_to)
        self._msg_type = msg_type
        self._last_published = rospy.get_time()
        self._velocity_mode = (getattr(msg_type, '_type', None) ==
                               'geometry_msgs/Twist')

        if self._velocity_mode:
            self._max_linear = _config_float(max_linear, '~max_linear',
                                             MAX_LINEAR, allow_zero=True)
            self._max_angular = _config_float(max_angular, '~max_angular',
                                              MAX_ANGULAR, allow_zero=True)
            self._linear_accel = _config_float(max_linear_accel,
                                               '~max_linear_accel', MAX_LINEAR_ACCEL)
            self._angular_accel = _config_float(max_angular_accel,
                                                '~angular_accel', MAX_ANGULAR_ACCEL)
            self._smoothing_tau = _config_float(smoothing_tau,
                                                '~smoothing_tau', SMOOTHING_TAU,
                                                allow_zero=True)
            self._cmd_timeout = _config_float(cmd_timeout,
                                              '~cmd_timeout', CMD_TIMEOUT)
            hz = DEFAULT_VELOCITY_HZ if frequency is None else float(frequency)
            if math.isnan(hz) or math.isinf(hz) or hz <= 0.0:
                raise ValueError('frequency must be a positive finite number')
            self._period = 1.0 / hz
            self._lock = threading.Lock()
            self._last_update = rospy.get_time()
            self._last_cmd_at = None
            self._target_linear = [0.0, 0.0, 0.0]
            self._target_angular = [0.0, 0.0, 0.0]
            self._filtered_linear = [0.0, 0.0, 0.0]
            self._filtered_angular = [0.0, 0.0, 0.0]
            self._output_linear = [0.0, 0.0, 0.0]
            self._output_angular = [0.0, 0.0, 0.0]
            # Timer memastikan output bergerak bertahap, walau ROS input
            # hanya mengirim satu perintah. Watchdog tetap aktif saat input hilang.
            self._timer = rospy.Timer(
                rospy.Duration.from_sec(self._period), self._velocity_timer_cb)
        else:
            self._interval = 0 if frequency is None else 1.0 / frequency

        rospy.Subscriber(topic_from, msg_type, self._callback_ros)

    def _callback_ros(self, msg):
        rospy.logdebug('ROS received from {}'.format(self._topic_from))
        if self._velocity_mode:
            linear = _clamp_vector(_read_vector(msg.linear), self._max_linear)
            angular = _clamp_vector(_read_vector(msg.angular), self._max_angular)
            with self._lock:
                self._target_linear = linear
                self._target_angular = angular
                self._last_cmd_at = rospy.get_time()
            return

        # Perilaku lama dipertahankan untuk selain geometry_msgs/Twist.
        now = rospy.get_time()
        if now - self._last_published >= self._interval:
            self._publish(msg)
            self._last_published = now

    def _velocity_timer_cb(self, event):
        now = rospy.get_time()
        with self._lock:
            dt = now - self._last_update
            self._last_update = now
            # Hindari lompatan output jika timer tersendat / ROS time berubah.
            dt = max(0.0, min(dt, 2.0 * self._period))

            stale = (self._last_cmd_at is None or
                     now < self._last_cmd_at or
                     now - self._last_cmd_at > self._cmd_timeout)

            if stale:
                # Watchdog: saat kontrol hilang, kirim 0 tanpa menunggu ramp.
                self._filtered_linear = [0.0, 0.0, 0.0]
                self._filtered_angular = [0.0, 0.0, 0.0]
                self._output_linear = [0.0, 0.0, 0.0]
                self._output_angular = [0.0, 0.0, 0.0]
            else:
                alpha = (1.0 if self._smoothing_tau == 0.0 else
                         1.0 - math.exp(-dt / self._smoothing_tau))
                self._filtered_linear = [
                    old + alpha * (wanted - old)
                    for old, wanted in zip(self._filtered_linear, self._target_linear)]
                self._filtered_angular = [
                    old + alpha * (wanted - old)
                    for old, wanted in zip(self._filtered_angular, self._target_angular)]
                self._output_linear = _clamp_vector(
                    _slew_vector(self._output_linear, self._filtered_linear,
                                 self._linear_accel, dt), self._max_linear)
                self._output_angular = _clamp_vector(
                    _slew_vector(self._output_angular, self._filtered_angular,
                                 self._angular_accel, dt), self._max_angular)

            out = self._msg_type()
            _write_vector(out.linear, self._output_linear)
            _write_vector(out.angular, self._output_angular)

        self._publish(out)

    def _publish(self, msg):
        if self._velocity_mode:
            # Lapis pengaman terakhir sebelum serialisasi MQTT.
            # Panggilan langsung ke _publish() juga tidak bisa melewati MAX.
            safe_msg = self._msg_type()
            _write_vector(safe_msg.linear,
                          _clamp_vector(_read_vector(msg.linear), self._max_linear))
            _write_vector(safe_msg.angular,
                          _clamp_vector(_read_vector(msg.angular), self._max_angular))
            msg = safe_msg
        payload = bytearray(self._serialize(extract_values(msg)))
        self._mqtt_client.publish(topic=self._topic_to, payload=payload)


class MqttToRosBridge(Bridge):
    u"""Bridge from MQTT topic to ROS topic. Kept unchanged."""

    def __init__(self, topic_from, topic_to, msg_type, frequency=None,
                 queue_size=10):
        self._topic_from = self._extract_private_path(topic_from)
        self._topic_to = topic_to
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._last_published = rospy.get_time()
        self._interval = None if frequency is None else 1.0 / frequency
        self._mqtt_client.subscribe(self._topic_from)
        self._mqtt_client.message_callback_add(self._topic_from, self._callback_mqtt)
        self._publisher = rospy.Publisher(
            self._topic_to, self._msg_type, queue_size=self._queue_size)

    def _callback_mqtt(self, client, userdata, mqtt_msg):
        rospy.logdebug('MQTT received from {}'.format(mqtt_msg.topic))
        now = rospy.get_time()
        if self._interval is None or now - self._last_published >= self._interval:
            try:
                ros_msg = self._create_ros_message(mqtt_msg)
                self._publisher.publish(ros_msg)
                self._last_published = now
            except Exception as e:
                rospy.logerr(e)

    def _create_ros_message(self, mqtt_msg):
        msg_dict = self._deserialize(mqtt_msg.payload)
        if self._msg_type._type in ('thouzer_msgs/Odom', 'thouzer_msgs/Vel'):
            if isinstance(msg_dict, dict):
                msg_dict.pop('timestamp', None)
        return populate_instance(msg_dict, self._msg_type())


__all__ = ['create_bridge', 'Bridge', 'RosToMqttBridge', 'MqttToRosBridge']
