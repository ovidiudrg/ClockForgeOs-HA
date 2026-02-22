import 'package:flutter/material.dart';

import 'screens/home_screen.dart';

void main() {
  runApp(const ClockForgeMobileApp());
}

class ClockForgeMobileApp extends StatelessWidget {
  const ClockForgeMobileApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'ClockForge Mobile',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xFF1E7A6B)),
        useMaterial3: true,
      ),
      home: const HomeScreen(),
    );
  }
}
