class CurrentInfos {
  final String currentDate;
  final String currentTime;
  final String timeSource;
  final double? temperature1;
  final double? humidity1;
  final double? pressure;
  final int? rssi;
  final bool tubesPower;
  final String dayNightMode;

  const CurrentInfos({
    required this.currentDate,
    required this.currentTime,
    required this.timeSource,
    required this.temperature1,
    required this.humidity1,
    required this.pressure,
    required this.rssi,
    required this.tubesPower,
    required this.dayNightMode,
  });

  static double? _asOptionalSensor(dynamic value) {
    if (value is num) {
      if (value == 255) return null;
      return value.toDouble();
    }
    return null;
  }

  factory CurrentInfos.fromJson(Map<String, dynamic> json) {
    return CurrentInfos(
      currentDate: (json['currentDate'] ?? '').toString(),
      currentTime: (json['currentTime'] ?? '').toString(),
      timeSource: (json['timeSource'] ?? 'Unknown').toString(),
      temperature1: _asOptionalSensor(json['temperature1'] ?? json['temperature']),
      humidity1: _asOptionalSensor(json['humidity1'] ?? json['humidity']),
      pressure: _asOptionalSensor(json['pressure']),
      rssi: json['rssi'] is num ? (json['rssi'] as num).toInt() : null,
      tubesPower: json['tubesPower'] == true,
      dayNightMode: (json['dayNightMode'] ?? 'UNKNOWN').toString(),
    );
  }
}
