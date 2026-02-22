import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;

import '../models/current_infos.dart';

class ReloginEvent {
  const ReloginEvent({
    required this.seq,
    required this.success,
    required this.message,
  });

  final int seq;
  final bool success;
  final String message;
}

class ClockForgeApi {
  ClockForgeApi({
    required this.baseUrl,
    http.Client? client,
  }) : _client = client ?? http.Client();

  final String baseUrl;
  final http.Client _client;
  String? _token;
  String? _password;
  final ValueNotifier<bool> isReloginInProgress = ValueNotifier<bool>(false);
  final ValueNotifier<ReloginEvent?> reloginEvent = ValueNotifier<ReloginEvent?>(
    null,
  );
  Future<void>? _ongoingRelogin;
  int _reloginEventSeq = 0;

  Uri _uri(String path) => Uri.parse('$baseUrl$path');

  Map<String, String> _authHeaders() {
    final token = _token;
    if (token == null || token.isEmpty) return {};
    return {'X-Auth-Token': token};
  }

  void _emitReloginEvent({
    required bool success,
    required String message,
  }) {
    _reloginEventSeq++;
    reloginEvent.value = ReloginEvent(
      seq: _reloginEventSeq,
      success: success,
      message: message,
    );
  }

  Future<void> login(String password) async {
    final res = await _client.post(
      _uri('/auth/login'),
      headers: {'Content-Type': 'application/x-www-form-urlencoded'},
      body: {'password': password},
    );

    if (res.statusCode != 200) {
      throw Exception('Login failed (${res.statusCode}): ${res.body}');
    }

    final data = jsonDecode(res.body) as Map<String, dynamic>;
    final token = data['token']?.toString();
    if (token == null || token.isEmpty) {
      throw Exception('Missing auth token');
    }
    _token = token;
    _password = password;
  }

  Future<void> _reloginAndRetry401() async {
    final running = _ongoingRelogin;
    if (running != null) {
      await running;
      return;
    }

    final password = _password;
    if (password == null || password.isEmpty) {
      throw Exception('Unauthorized. Please login again.');
    }

    Future<void> doRelogin() async {
      isReloginInProgress.value = true;
      _token = null;
      try {
        await login(password);
        _emitReloginEvent(
          success: true,
          message: 'Session refreshed.',
        );
      } catch (_) {
        _emitReloginEvent(
          success: false,
          message: 'Session refresh failed. Please login again.',
        );
        rethrow;
      } finally {
        isReloginInProgress.value = false;
      }
    }

    final reloginFuture = doRelogin();
    _ongoingRelogin = reloginFuture;
    try {
      await reloginFuture;
    } finally {
      if (identical(_ongoingRelogin, reloginFuture)) {
        _ongoingRelogin = null;
      }
    }
  }

  Future<http.Response> _getWithAutoRelogin(String path) async {
    var res = await _client.get(_uri(path), headers: _authHeaders());
    if (res.statusCode != 401) return res;
    await _reloginAndRetry401();
    res = await _client.get(_uri(path), headers: _authHeaders());
    return res;
  }

  Future<http.Response> _postFormWithAutoRelogin(
    String path, {
    required Map<String, String> body,
  }) async {
    Future<http.Response> doPost() {
      return _client.post(
        _uri(path),
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
          ..._authHeaders(),
        },
        body: body,
      );
    }

    var res = await doPost();
    if (res.statusCode != 401) return res;
    await _reloginAndRetry401();
    res = await doPost();
    return res;
  }

  Future<CurrentInfos> getCurrentInfos() async {
    final res = await _client.get(_uri('/getCurrentInfos'));
    if (res.statusCode != 200) {
      throw Exception('getCurrentInfos failed (${res.statusCode})');
    }
    return CurrentInfos.fromJson(jsonDecode(res.body) as Map<String, dynamic>);
  }

  Future<Map<String, dynamic>> getConfiguration() async {
    final res = await _getWithAutoRelogin('/getConfiguration');
    if (res.statusCode == 401) {
      throw Exception('Unauthorized. Please login again.');
    }
    if (res.statusCode != 200) {
      throw Exception('getConfiguration failed (${res.statusCode})');
    }
    return jsonDecode(res.body) as Map<String, dynamic>;
  }

  Future<void> saveSetting({
    required String key,
    required String value,
  }) async {
    final res = await _postFormWithAutoRelogin(
      '/saveSetting',
      body: {'key': key, 'value': value},
    );

    if (res.statusCode == 401) {
      throw Exception('Unauthorized. Please login again.');
    }
    if (res.statusCode != 200) {
      throw Exception('saveSetting failed (${res.statusCode}): ${res.body}');
    }
  }

  Future<void> setManualTimeEpoch(int epochSeconds) async {
    final res = await _postFormWithAutoRelogin(
      '/setManualTime',
      body: {'epoch': '$epochSeconds'},
    );

    if (res.statusCode == 401) {
      throw Exception('Unauthorized. Please login again.');
    }
    if (res.statusCode != 200) {
      throw Exception('setManualTime failed (${res.statusCode}): ${res.body}');
    }
  }

  void dispose() {
    isReloginInProgress.dispose();
    reloginEvent.dispose();
    _client.close();
  }
}
