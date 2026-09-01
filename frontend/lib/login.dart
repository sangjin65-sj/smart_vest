import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'home.dart';

class LoginScreen extends StatefulWidget {
  const LoginScreen({super.key});

  @override
  State<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends State<LoginScreen> {
  // 1단계(보호자 로그인) 컨트롤러
  final _guardianNameController = TextEditingController();
  final _guardianPhoneController = TextEditingController();

  // 2단계(보호 대상자 정보) 컨트롤러
  final _patientNameController = TextEditingController();
  final _patientAgeController = TextEditingController();

  // 화면 전환 관리 (false: 보호자 입력, true: 보호 대상자 입력)
  bool _isGuardianDone = false;
  bool _isLoading = true; // 자동 로그인 확인 중 로딩 상태

  final _guardianFormKey = GlobalKey<FormState>();
  final _patientFormKey = GlobalKey<FormState>();

  @override
  void initState() {
    super.initState();
    _checkSavedLogin();
  }

  // 로컬 저장소에 저장된 로그인 정보 확인 (자동 로그인 처리)
  Future<void> _checkSavedLogin() async {
    final prefs = await SharedPreferences.getInstance();
    final isLoggedIn = prefs.getBool('isLoggedIn') ?? false;

    if (isLoggedIn && mounted) {
      final guardianName = prefs.getString('guardianName') ?? '';
      final patientName = prefs.getString('patientName') ?? '';
      final patientAge = prefs.getString('patientAge') ?? '';

      // MainScreen으로 이동하도록 변경
      Navigator.pushReplacement(
        context,
        MaterialPageRoute(
          builder: (context) => MainScreen(
            guardianName: guardianName,
            patientName: patientName,
            patientAge: patientAge,
          ),
        ),
      );
    } else {
      setState(() {
        _isLoading = false; // 저장된 정보가 없으면 로그인 화면 표시
      });
    }
  }

  @override
  void dispose() {
    _guardianNameController.dispose();
    _guardianPhoneController.dispose();
    _patientNameController.dispose();
    _patientAgeController.dispose();
    super.dispose();
  }

  // 1단계: 보호자 정보 입력 완료
  void _handleGuardianLogin() {
    if (_guardianFormKey.currentState!.validate()) {
      setState(() {
        _isGuardianDone = true; // 2단계 입력 화면으로 전환
      });
    }
  }

  // 2단계: 보호 대상자 입력 및 데이터 최종 저장 (자동 로그인 설정)
  Future<void> _handleFinalSubmit() async {
    if (_patientFormKey.currentState!.validate()) {
      final guardianName = _guardianNameController.text;
      final guardianPhone = _guardianPhoneController.text;
      final patientName = _patientNameController.text;
      final patientAge = _patientAgeController.text;

      // SharedPreferences에 데이터 저장
      final prefs = await SharedPreferences.getInstance();
      await prefs.setBool('isLoggedIn', true);
      await prefs.setString('guardianName', guardianName);
      await prefs.setString('guardianPhone', guardianPhone);
      await prefs.setString('patientName', patientName);
      await prefs.setString('patientAge', patientAge);

      if (mounted) {
        // MainScreen으로 이동하도록 변경
        Navigator.pushReplacement(
          context,
          MaterialPageRoute(
            builder: (context) => MainScreen(
              guardianName: guardianName,
              patientName: patientName,
              patientAge: patientAge,
            ),
          ),
        );
      }
    }
  }

  @override
  Widget build(BuildContext context) {
    if (_isLoading) {
      return const Scaffold(
        body: Center(
          child: CircularProgressIndicator(),
        ),
      );
    }

    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: Text(_isGuardianDone ? '보호 대상(조끼 사용자) 정보 입력' : '보호자 로그인'),
        centerTitle: true,
        backgroundColor: Colors.blueAccent,
        foregroundColor: Colors.white,
      ),
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(24.0),
          child: !_isGuardianDone
              ? Form(
            key: _guardianFormKey,
            child: ListView(
              children: [
                const SizedBox(height: 20),
                const Text(
                  '보호자 본인 정보',
                  style: TextStyle(
                    fontSize: 24,
                    fontWeight: FontWeight.bold,
                    color: Colors.black87,
                  ),
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 10),
                const Text(
                  '앱을 관리할 보호자님의 성함과 연락처를 입력해 주세요.',
                  style: TextStyle(fontSize: 14, color: Colors.grey),
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 40),

                // [1단계] 보호자 이름 입력
                TextFormField(
                  controller: _guardianNameController,
                  decoration: const InputDecoration(
                    labelText: '보호자 이름',
                    border: OutlineInputBorder(),
                    prefixIcon: Icon(Icons.person),
                  ),
                  validator: (value) {
                    if (value == null || value.trim().isEmpty) {
                      return '보호자 이름을 입력해주세요.';
                    }
                    return null;
                  },
                ),
                const SizedBox(height: 20),

                // [1단계] 보호자 전화번호 입력
                TextFormField(
                  controller: _guardianPhoneController,
                  keyboardType: TextInputType.phone,
                  decoration: const InputDecoration(
                    labelText: '보호자 전화번호',
                    hintText: '010-0000-0000',
                    border: OutlineInputBorder(),
                    prefixIcon: Icon(Icons.phone),
                  ),
                  validator: (value) {
                    if (value == null || value.trim().isEmpty) {
                      return '보호자 전화번호를 입력해주세요.';
                    }
                    return null;
                  },
                ),
                const SizedBox(height: 40),

                // 보호자 로그인 버튼
                ElevatedButton(
                  onPressed: _handleGuardianLogin,
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.blueAccent,
                    foregroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(vertical: 16),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(8),
                    ),
                  ),
                  child: const Text(
                    '다음 (보호 대상자 입력)',
                    style: TextStyle(
                        fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                ),
              ],
            ),
          )
              : Form(
            key: _patientFormKey,
            child: ListView(
              children: [
                const SizedBox(height: 20),
                const Text(
                  '조끼 사용자 정보',
                  style: TextStyle(
                    fontSize: 24,
                    fontWeight: FontWeight.bold,
                    color: Colors.black87,
                  ),
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 10),
                const Text(
                  '실종 방지 조끼와 연동할 사용자의 이름과 나이를 입력해 주세요.',
                  style: TextStyle(fontSize: 14, color: Colors.grey),
                  textAlign: TextAlign.center,
                ),
                const SizedBox(height: 40),

                // [2단계] 보호 대상자 이름 입력
                TextFormField(
                  controller: _patientNameController,
                  decoration: const InputDecoration(
                    labelText: '사용자 이름',
                    border: OutlineInputBorder(),
                    prefixIcon: Icon(Icons.elderly),
                  ),
                  validator: (value) {
                    if (value == null || value.trim().isEmpty) {
                      return '사용자 이름을 입력해주세요.';
                    }
                    return null;
                  },
                ),
                const SizedBox(height: 20),

                // [2단계] 보호 대상자 나이 입력
                TextFormField(
                  controller: _patientAgeController,
                  keyboardType: TextInputType.number,
                  decoration: const InputDecoration(
                    labelText: '나이',
                    border: OutlineInputBorder(),
                    prefixIcon: Icon(Icons.cake),
                  ),
                  validator: (value) {
                    if (value == null || value.trim().isEmpty) {
                      return '사용자의 나이를 입력해주세요.';
                    }
                    if (int.tryParse(value) == null) {
                      return '올바른 숫자를 입력해주세요.';
                    }
                    return null;
                  },
                ),
                const SizedBox(height: 40),

                // 최종 시작하기 버튼
                ElevatedButton(
                  onPressed: _handleFinalSubmit,
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.green,
                    foregroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(vertical: 16),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(8),
                    ),
                  ),
                  child: const Text(
                    '조끼 연동 및 시작하기',
                    style: TextStyle(
                        fontSize: 18, fontWeight: FontWeight.bold),
                  ),
                ),
                const SizedBox(height: 12),

                // 이전 단계로 돌아가기 버튼
                TextButton(
                  onPressed: () {
                    setState(() {
                      _isGuardianDone = false;
                    });
                  },
                  child: const Text('보호자 정보 다시 입력하기'),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}