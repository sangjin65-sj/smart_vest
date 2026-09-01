import 'package:flutter/material.dart';
import 'package:flutter_local_notifications/flutter_local_notifications.dart';

final FlutterLocalNotificationsPlugin flutterLocalNotificationsPlugin =
FlutterLocalNotificationsPlugin();

Future<void> initLocalNotifications() async {
  const AndroidInitializationSettings initializationSettingsAndroid =
  AndroidInitializationSettings('@mipmap/ic_launcher');

  const InitializationSettings initializationSettings =
  InitializationSettings(android: initializationSettingsAndroid);

  await flutterLocalNotificationsPlugin.initialize(
    initializationSettings,
    onDidReceiveNotificationResponse: (NotificationResponse response) {
      // 알림 클릭 시 실행될 동적인 행위
    },
  );

  final androidPlugin = flutterLocalNotificationsPlugin
      .resolvePlatformSpecificImplementation<
      AndroidFlutterLocalNotificationsPlugin>();

  if (androidPlugin != null) {
    await androidPlugin.requestNotificationsPermission();
  }

  // 💡 상단 헤드업 팝업을 보장하기 위한 최고 중요도 알림 채널 정의
  const AndroidNotificationChannel channel = AndroidNotificationChannel(
    'emergency_channel_v4',
    '응급 비상 알림',
    description: '낙상 감지 및 안심구역/이동경로 이탈 비상 알림을 전달합니다.',
    importance: Importance.max,
    playSound: true,
    enableVibration: true,
  );

  await androidPlugin?.createNotificationChannel(channel);
}

Future<void> triggerNotification(int id, String title, String body) async {
  const AndroidNotificationDetails androidDetails = AndroidNotificationDetails(
    'emergency_channel_v4',
    '응급 비상 알림',
    channelDescription: '낙상 감지 및 안심구역/이동경로 이탈 비상 알림을 전달합니다.',
    importance: Importance.max,
    priority: Priority.high,
    fullScreenIntent: true,
    showWhen: true,
    color: Colors.red,
  );

  const NotificationDetails platformDetails =
  NotificationDetails(android: androidDetails);

  await flutterLocalNotificationsPlugin.show(
    id,
    title,
    body,
    platformDetails,
  );
}