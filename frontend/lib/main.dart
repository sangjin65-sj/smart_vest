import 'package:flutter/material.dart';
import 'package:firebase_core/firebase_core.dart';
import 'login.dart';
import 'notification_helper.dart'; // 💡 알림 헬퍼 불러오기

void main() async {
  // Flutter 엔진 및 Firebase 초기화
  WidgetsFlutterBinding.ensureInitialized();
  await Firebase.initializeApp();

  // 💡 [필수] 알림 채널 및 로컬 알림 서비스 초기화
  await initLocalNotifications();

  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: '치매노인 조끼 연동 시스템',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: const LoginScreen(),
    );
  }
}