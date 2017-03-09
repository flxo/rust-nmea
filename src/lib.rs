// Copyright (C) 2016 Felix Obenhuber
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#[macro_use]
extern crate lazy_static;
extern crate regex;
extern crate chrono;

use chrono::{DateTime, UTC, Date};
use chrono::naive::time::NaiveTime;
use chrono::naive::date::NaiveDate;
use chrono::naive::datetime::NaiveDateTime;
use regex::Regex;
use std::collections::HashMap;
use std::fmt;
use std::vec::Vec;

/// NMEA parser
#[derive(Default)]
pub struct Nmea {
    pub fix_timestamp_time: Option<NaiveTime>,
    pub fix_timestamp_date: Option<Date<UTC>>,
    pub fix_type: Option<FixType>,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub altitude: Option<f32>,
    pub fix_satellites: Option<u32>,
    pub hdop: Option<f32>,
    pub geoid_height: Option<f32>,
    pub speed_over_ground: Option<f32>,
    pub true_course: Option<f32>,
    pub satellites: Vec<Satellite>,
    satellites_scan: HashMap<GnssType, Vec<Vec<Satellite>>>,
}

#[derive(Debug)]
pub enum ParseResult {
    GGA(GgaData),
    RMC(RmcData),
    GSV(GsvData),
    Unsupported(SentenceType),
}

impl<'a> Nmea {
    /// Constructs a new `Nmea`.
    /// This struct parses NMEA sentences, including checksum checks and sentence
    /// validation.
    ///
    /// # Examples
    ///
    /// ```
    /// use nmea::Nmea;
    ///
    /// let mut nmea= Nmea::new();
    /// let gga = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76";
    /// nmea.parse(gga).unwrap();
    /// println!("{}", nmea);
    /// ```
    pub fn new() -> Nmea {
        let mut n = Self::default();
        n.satellites_scan.insert(GnssType::Galileo, vec![]);
        n.satellites_scan.insert(GnssType::Gps, vec![]);
        n.satellites_scan.insert(GnssType::Glonass, vec![]);
        n
    }

    /// Returns fix type
    pub fn fix_timestamp_time(&self) -> Option<NaiveTime> {
        self.fix_timestamp_time
    }

    /// Returns fix type
    pub fn fix_type(&self) -> Option<FixType> {
        self.fix_type.clone()
    }

    /// Returns last fixed latitude in degress. None if not fixed.
    pub fn latitude(&self) -> Option<f32> {
        self.latitude
    }

    /// Returns last fixed longitude in degress. None if not fixed.
    pub fn longitude(&self) -> Option<f32> {
        self.longitude
    }

    /// Returns latitude from last fix. None if not available.
    pub fn altitude(&self) -> Option<f32> {
        self.altitude
    }

    /// Returns the number of satellites use for fix.
    pub fn fix_satellites(&self) -> Option<u32> {
        self.fix_satellites
    }

    /// Returns the number fix HDOP
    pub fn hdop(&self) -> Option<f32> {
        self.hdop
    }

    /// Returns the height of geoid above WGS84
    pub fn geoid_height(&self) -> Option<f32> {
        self.geoid_height
    }

    /// Returns the height of geoid above WGS84
    pub fn satellites(&self) -> Vec<Satellite> {
        self.satellites.clone()
    }

    /// Returns the NMEA sentence type.
    pub fn sentence_type(&self, s: &'a str) -> Result<SentenceType, &'static str> {
        match REGEX_TYPE.captures(s) {
            Some(c) => {
                match c.name("type") {
                    Some(s) => {
                        match SentenceType::from(s.as_str()) {
                            SentenceType::None => Err("Unknown type"),
                            _ => Ok(SentenceType::from(s.as_str())),
                        }
                    }
                    _ => Err("Failed to parse type"),
                }
            }
            None => Err("Failed to parse type"),
        }
    }

    pub fn parse(&mut self, s: &'a str) -> Result<SentenceType, &'static str> {
        match self.do_parse(s)? {
            ParseResult::GGA(gga_data) => {
                self.fix_timestamp_time = gga_data.fix_timestamp_time;
                self.latitude = gga_data.latitude;
                self.longitude = gga_data.longitude;
                self.fix_type = gga_data.fix_type;
                self.fix_satellites = gga_data.fix_satellites;
                self.hdop = gga_data.hdop;
                self.altitude = gga_data.altitude;
                self.geoid_height = gga_data.geoid_height;
                Ok(SentenceType::GGA)
            }
            ParseResult::RMC(rmc_data) => {
                self.fix_timestamp_time = rmc_data.fix_time.map(|v| v.time());
                self.fix_timestamp_date = rmc_data.fix_time.map(|v| v.date());
                self.fix_type = rmc_data.status_of_fix
                    .map(|v| match v {
                        RmcStatusOfFix::Autonomous => FixType::Gps,
                        RmcStatusOfFix::Differential => FixType::DGps,
                        RmcStatusOfFix::Invalid => FixType::Invalid,
                    });
                self.latitude = rmc_data.lat;
                self.longitude = rmc_data.lon;
                self.speed_over_ground = rmc_data.speed_over_ground;
                self.true_course = rmc_data.true_course;
                Ok(SentenceType::RMC)
            }
            ParseResult::GSV(gsv_data) => {
                {
                    let d = self.satellites_scan.get_mut(&gsv_data.gnss_type).ok_or("Invalid GNSS type")?;
                    // Adjust size to this scan
                    d.resize(gsv_data.number, vec![]);
                    // Replace data at index with new scan data
                    d.push(gsv_data.sats);
                    d.swap_remove(gsv_data.index - 1);
                }
                self.satellites.clear();
                for (_, v) in &self.satellites_scan {
                    for v1 in v {
                        for v2 in v1 {
                            self.satellites.push(v2.clone());
                        }
                    }
                }
                Ok(SentenceType::GSV)
            }
            ParseResult::Unsupported(_) => return Err("Unsupported sentence type"),
        }
    }

    /// Parse any NMEA sentence and stores the result. The type of sentence
    /// is returnd if implemented and valid.
    pub fn do_parse(&mut self, s: &'a str) -> Result<ParseResult, &'static str> {
        if Nmea::checksum(s)? {
            match self.sentence_type(&s)? {
                SentenceType::RMC => {
                    let rmc_data = parse_rmc(s)?;

                    Ok(ParseResult::RMC(rmc_data))
                }
                SentenceType::GGA => {
                    let gga_data = parse_gga(s)?;
                    Ok(ParseResult::GGA(gga_data))
                }
                SentenceType::GSV => {
                    let gsv_data = parse_gsv(s)?;
                    Ok(ParseResult::GSV(gsv_data))
                }
                t => Ok(ParseResult::Unsupported(t)),
            }
        } else {
            Err("Checksum mismatch")
        }
    }

    fn checksum(s: &str) -> Result<bool, &'static str> {
        let caps = REGEX_CHECKSUM.captures(s).ok_or("Failed to parse sentence")?;
        let sentence = caps.name(&"sentence").ok_or("Failed to parse sentence")?;
        let checksum =
            caps.name(&"checksum")
                .ok_or("Failed to parse checksum")
                .and_then(|c| {
                    u8::from_str_radix(c.as_str(), 16).map_err(|_| "Failed to parse checksun")
                })?;
        Ok(checksum == sentence.as_str().bytes().fold(0, |c, x| c ^ x))
    }
}

macro_rules! map_not_empty {
    ($StrName: ident, $Expr: expr) => {
        if !$StrName.is_empty() {
            Some($Expr)
        } else {
            None
        }
    }
}

#[derive(Debug)]
pub enum RmcStatusOfFix {
    Autonomous,
    Differential,
    Invalid,
}

#[derive(Debug)]
pub struct RmcData {
    pub fix_time: Option<DateTime<UTC>>,
    pub status_of_fix: Option<RmcStatusOfFix>,
    pub lat: Option<f32>,
    pub lon: Option<f32>,
    pub speed_over_ground: Option<f32>,
    pub true_course: Option<f32>,
}

fn parse_rmc(input: &str) -> Result<RmcData, &'static str> {
    let mut field = input.split(",");
    if !REGEX_RMC_HEAD.is_match(field.next().ok_or("parse_rmc failed: non RMC type")?) {
        return Err("Not start with $..RMC");
    }
    let time = field.next().ok_or("parse_rmc failed: no time")?;
    let time: Option<NaiveTime> = map_not_empty!(time, {
        parse_hms(time)?
    });

    let status_of_fix = field.next().ok_or("parse_rmc failed: no status of fix")?;
    let status_of_fix = map_not_empty!(status_of_fix, match status_of_fix {
        "A" => RmcStatusOfFix::Autonomous,
        "D" => RmcStatusOfFix::Differential,
        "V" => RmcStatusOfFix::Invalid,
        _ => { return Err("parse_rmc failed: not A|D|V status of fix"); }
    });

    let (lat, lon) = parse_lat_lon(&mut field)?;

    let speed_over_ground = field.next().ok_or("parse_rmc failed: no speed over ground")?;
    let speed_over_ground = map_not_empty!(speed_over_ground, parse_numeric::<f32>(speed_over_ground, 1.0)?);
    let course = field.next().ok_or("parse_rmc failed: no course")?;
    let course = map_not_empty!(course, parse_numeric::<f32>(course, 1.0)?);
    let date = field.next().ok_or("parse_rmc failed: no date of fix")?;
    let date: Option<NaiveDate> = map_not_empty!(date, {
        if date.len() != 6 {
            return Err("Length string with date of fix not 6");
        } else {
            let day =  parse_numeric::<u8>(&date[0..2], 1)?;
            let month = parse_numeric::<u8>(&date[2..4], 1)?;
            let year = parse_numeric::<u8>(&date[4..6], 1)?;
            NaiveDate::from_ymd(year as i32, month as u32, day as u32)
        }
    });

    Ok(RmcData {
        fix_time: if let (Some(date), Some(time)) = (date, time) {
            Some(DateTime::from_utc(NaiveDateTime::new(date, time), UTC))
        } else {
            None
        },
        status_of_fix: status_of_fix,
        lat: lat,
        lon: lon,
        speed_over_ground: speed_over_ground,
        true_course: course,
    })
}

#[derive(Debug)]
pub struct GgaData {
    pub fix_timestamp_time: Option<NaiveTime>,
    pub fix_type: Option<FixType>,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub fix_satellites: Option<u32>,
    pub hdop: Option<f32>,
    pub altitude: Option<f32>,
    pub geoid_height: Option<f32>,
}

fn parse_gga(sentence: &str) -> Result<GgaData, &'static str> {
    let mut field = sentence.split(",");
    if !REGEX_GGA_HEAD.is_match(field.next().ok_or("no header")?) {
        return Err("Not start with $..GGA");
    }
    let time = field.next().ok_or("no time")?;
    let time: Option<NaiveTime> = map_not_empty!(time, parse_hms(time)?);
    let (lat, lon) = parse_lat_lon(&mut field)?;
    let fix_type = field.next().ok_or("no fix quality")?;
    let fix_type = map_not_empty!(fix_type, FixType::from(fix_type));

    let fix_satellites = field.next().ok_or("no fix satellites field")?;
    let fix_satellites = map_not_empty!(fix_satellites,
                                        parse_numeric::<u32>(fix_satellites, 1)?);
    let hdop = field.next().ok_or("no hdop feild")?;
    let hdop = map_not_empty!(hdop, parse_numeric::<f32>(hdop, 1.0)?);

    let altitude = field.next().ok_or("no alt")?;
    let altitude_type = field.next().ok_or("no alt type")?;
    let altitude = map_not_empty!(altitude, {
        if altitude_type != "M" {
            return Err("no supported altitude type, should be 'M'");
        }
        parse_numeric::<f32>(altitude, 1.0)?
    });

    let geoid_height = field.next().ok_or("no geoid height")?;
    let geoid_height_type = field.next().ok_or("no geoid height type")?;
    let geoid_height = map_not_empty!(geoid_height, {
        if geoid_height_type != "M" {
            return Err("no supported geoid height type, should be 'M'");
        }
        parse_numeric::<f32>(geoid_height, 1.0)?
    });

    Ok(GgaData {
        fix_timestamp_time: time,
        latitude: lat,
        longitude: lon,
        fix_type: fix_type,
        fix_satellites: fix_satellites,
        hdop: hdop,
        altitude: altitude,
        geoid_height: geoid_height,
    })
}

#[derive(Debug)]
pub struct GsvData {
    gnss_type: GnssType,
    number: usize,
    index: usize,
    sats: Vec<Satellite>,
}

fn parse_gsv(sentence: &str) -> Result<GsvData, &'static str> {
    match REGEX_GSV.captures(sentence) {
        Some(caps) => {
            let gnss_type = match caps.name("type") {
                Some(t) => {
                    match t.as_str() {
                        "GP" => GnssType::Gps,
                        "GL" => GnssType::Glonass,
                        _ => return Err("Unknown GNSS type in GSV sentence"),
                    }
                }
                None => return Err("Failed to parse GSV sentence"),
            };
            let number = caps.name("number")
                .ok_or("Failed to parse number of sats")
                .and_then(|n| parse_numeric::<usize>(n.as_str(), 1))?;
            let index = caps.name("index")
                .ok_or("Failed to parse satellite index")
                .and_then(|n| parse_numeric::<usize>(n.as_str(), 1))?;

            let sats = caps.name("sats")
                .ok_or("Failed to parse sats")
                .and_then(|s| parse_satellites(s.as_str(), &gnss_type))?;


            Ok(GsvData {
                gnss_type: gnss_type,
                number: number,
                index: index,
                sats: sats,
            })
        }
        None => Err("Failed to parse GSV sentence"),
    }
}


fn parse_numeric<T>(input: &str, factor: T) -> Result<T, &'static str>
    where T: std::str::FromStr + std::ops::Mul<Output = T> + Copy
{
    input.parse::<T>().map(|v| v * factor).map_err(|_| "Failed to parse number")
}

/// Parse a HHMMSS[.milli] string into time
fn parse_hms(s: &str) -> Result<NaiveTime, &'static str> {
    if s.len() < 6 {
        return Err("Failed to parse time");
    } else {
        let hour = parse_numeric::<u32>(&s[0..2], 1)?;
        let min = parse_numeric::<u32>(&s[2..4], 1)?;
        let sec = parse_numeric::<f64>(&s[4..], 1.0)?;
        Ok(NaiveTime::from_hms_nano(hour, min, sec.floor() as u32,
                                    (sec.fract() * 1_000_000_000f64).round() as u32))
    }
}

fn parse_lat_lon<'a>(field: &mut Iterator<Item = &'a str>) -> Result<(Option<f32>, Option<f32>), &'static str> {
    let lat = field.next().ok_or("parse_rmc failed: no lattitude")?;
    let lat_dir = match field.next().ok_or("parse_rmc failed: no lattitude direction")? {
        "S" => -0.01,
        "N" => 0.01,
        _ => { return Err("parse_rmc failed: wrong lat dir, not of S|N"); }
    };
    let lat = map_not_empty!(lat, {
        let num = parse_numeric::<f32>(lat, lat_dir)?;
        num.round() + (num.fract() * 100.0) / 60.
    });

    let lon = field.next().ok_or("parse_rmc failed: no longitude")?;
    let lon_dir = match field.next().ok_or("parse_rmc failed: no longitude direction")? {
        "W" => -0.01,
        "E" => 0.01,
        _ => { return Err("parse_rmc failed: wrong lon dir"); }
    };
    let lon = map_not_empty!(lon, {
       let num = parse_numeric::<f32>(lon, lon_dir)?;
        num.round() + (num.fract() * 100.0) / 60.
    });

    Ok((lat, lon))
}

fn parse_satellites(satellites: &str,
                    gnss_type: &GnssType)
                    -> Result<Vec<Satellite>, &'static str> {
    let mut sats = vec![];
    let mut s = satellites.split(',');
    for _ in 0..3 {
        if let Some(prn) = s.next()
            .and_then(|a| parse_numeric::<u32>(a, 1).ok()) {
                sats.push(Satellite {
                    gnss_type: gnss_type.clone(),
                    prn: prn,
                    elevation: s.next()
                        .ok_or("Failed to parse elevation")
                        .and_then(|a| parse_numeric::<f32>(a, 1.0))
                        .ok(),
                    azimuth: s.next()
                        .ok_or("Failed to parse azimuth")
                        .and_then(|a| parse_numeric::<f32>(a, 1.0))
                        .ok(),
                    snr: s.next()
                        .ok_or("Failed to parse SNR")
                        .and_then(|a| parse_numeric::<f32>(a, 1.0))
                        .ok(),
                });
            } else {
                return Err("Failed to parse prn");
            }
    }

    Ok(sats)
}


impl fmt::Debug for Nmea {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl fmt::Display for Nmea {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
               "{}: lat: {} lon: {} alt: {} {:?}",
               self.fix_timestamp_time.map(|l| format!("{}", l)).unwrap_or("None".to_owned()),
               self.latitude.map(|l| format!("{:3.8}", l)).unwrap_or("None".to_owned()),
               self.longitude.map(|l| format!("{:3.8}", l)).unwrap_or("None".to_owned()),
               self.altitude.map(|l| format!("{:.3}", l)).unwrap_or("None".to_owned()),
               self.satellites())
    }
}

#[derive (Clone)]
/// ! A Satellite
pub struct Satellite {
    gnss_type: GnssType,
    prn: u32,
    elevation: Option<f32>,
    azimuth: Option<f32>,
    snr: Option<f32>,
}

impl Satellite {
    pub fn gnss_type(&self) -> GnssType {
        self.gnss_type.clone()
    }

    pub fn prn(&self) -> u32 {
        self.prn
    }

    pub fn elevation(&self) -> Option<f32> {
        self.elevation
    }

    pub fn azimuth(&self) -> Option<f32> {
        self.azimuth
    }

    pub fn snr(&self) -> Option<f32> {
        self.snr
    }
}

impl fmt::Display for Satellite {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
               "{}: {} elv: {} ath: {} snr: {}",
               self.gnss_type,
               self.prn,
               self.elevation.map(|e| format!("{}", e)).unwrap_or("--".to_owned()),
               self.azimuth.map(|e| format!("{}", e)).unwrap_or("--".to_owned()),
               self.snr.map(|e| format!("{}", e)).unwrap_or("--".to_owned()))
    }
}

impl fmt::Debug for Satellite {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
               "[{:?},{:?},{:?},{:?},{:?}]",
               self.gnss_type,
               self.prn,
               self.elevation,
               self.azimuth,
               self.snr)
    }
}

macro_rules! define_sentence_type_enum {
    ($Name:ident { $($Variant:ident),* }) => {
        #[derive(PartialEq, Debug)]
        pub enum $Name {
            None,
            $($Variant),*,
        }

        impl<'a> From<&'a str> for $Name {
            fn from(s: &str) -> Self {
                match s {
                    $(stringify!($Variant) => $Name::$Variant,)*
                    _ => $Name::None,
                }
            }
        }
    }
}

#[test]
fn test_define_sentence_type_enum() {
    define_sentence_type_enum!( TestEnum {
        AAA,
        BBB
    }
    );

    let a = TestEnum::AAA;
    let b = TestEnum::BBB;
    let n = TestEnum::None;
    assert_eq!(TestEnum::from("AAA"), a);
    assert_eq!(TestEnum::from("BBB"), b);
    assert_eq!(TestEnum::from("fdafa"), n);
}

/// ! NMEA sentence type
/// ! General: OSD |
/// ! Autopilot: APA | APB | ASD |
/// ! Decca: DCN |
/// ! D-GPS: MSK
/// ! Echo: DBK | DBS | DBT |
/// ! Radio: FSI | SFI | TLL
/// ! Speed: VBW | VHW | VLW |
/// ! GPS: ALM | GBS | GGA | GNS | GSA | GSV |
/// ! Course: DPT | HDG | HDM | HDT | HSC | ROT | VDR |
/// ! Loran-C: GLC | LCD |
/// ! Machine: RPM |
/// ! Navigation: RMA | RMB | RMC |
/// ! Omega: OLN |
/// ! Position: GLL | DTM
/// ! Radar: RSD | TLL | TTM |
/// ! Rudder: RSA |
/// ! Temperature: MTW |
/// ! Transit: GXA | RTF |
/// ! Waypoints and tacks: AAM | BEC | BOD | BWC | BWR | BWW | ROO | RTE | VTG | WCV | WNC | WPL | XDR | XTE | XTR |
/// ! Wind: MWV | VPW | VWR |
/// ! Date and Time: GDT | ZDA | ZFO | ZTG |
define_sentence_type_enum!(SentenceType {
    AAM,
    ABK,
    ACA,
    ACK,
    ACS,
    AIR,
    ALM,
    ALR,
    APA,
    APB,
    ASD,
    BEC,
    BOD,
    BWC,
    BWR,
    BWW,
    CUR,
    DBK,
    DBS,
    DBT,
    DCN,
    DPT,
    DSC,
    DSE,
    DSI,
    DSR,
    DTM,
    FSI,
    GBS,
    GGA,
    GLC,
    GLL,
    GMP,
    GNS,
    GRS,
    GSA,
    GST,
    GSV,
    GTD,
    GXA,
    HDG,
    HDM,
    HDT,
    HMR,
    HMS,
    HSC,
    HTC,
    HTD,
    LCD,
    LRF,
    LRI,
    LR1,
    LR2,
    LR3,
    MLA,
    MSK,
    MSS,
    MWD,
    MTW,
    MWV,
    OLN,
    OSD,
    ROO,
    RMA,
    RMB,
    RMC,
    ROT,
    RPM,
    RSA,
    RSD,
    RTE,
    SFI,
    SSD,
    STN,
    TLB,
    TLL,
    TRF,
    TTM,
    TUT,
    TXT,
    VBW,
    VDM,
    VDO,
    VDR,
    VHW,
    VLW,
    VPW,
    VSD,
    VTG,
    VWR,
    WCV,
    WNC,
    WPL,
    XDR,
    XTE,
    XTR,
    ZDA,
    ZDL,
    ZFO,
    ZTG
});

/// ! Fix type
#[derive(Clone, PartialEq, Debug)]
pub enum FixType {
    Invalid,
    Gps,
    DGps,
    Pps,
    Rtk,
    FloatRtk,
    Estimated,
    Manual,
    Simulation,
}

/// ! GNSS type
#[derive (Debug, Clone, Hash, Eq, PartialEq)]
pub enum GnssType {
    Galileo,
    Gps,
    Glonass,
}

impl fmt::Display for GnssType {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            GnssType::Galileo => write!(f, "Galileo"),
            GnssType::Gps => write!(f, "GPS"),
            GnssType::Glonass => write!(f, "GLONASS"),
        }
    }
}

impl<'a> From<&'a str> for FixType {
    fn from(s: &'a str) -> Self {
        match s {
            "Invaild" => FixType::Invalid,
            "Gps" => FixType::Gps,
            "DGps" => FixType::DGps,
            "Pps" => FixType::Pps,
            "Rtk" => FixType::Rtk,
            "FloatRtk" => FixType::FloatRtk,
            "Estimated" => FixType::Estimated,
            "Manual" => FixType::Manual,
            "Simulation" => FixType::Simulation,
            _ => {
                match parse_numeric::<u8>(s, 1) {
                    Ok(n) => {
                        match n {
                            0 => FixType::Invalid,
                            1 => FixType::Gps,
                            2 => FixType::DGps,
                            3 => FixType::Pps,
                            4 => FixType::Rtk,
                            5 => FixType::FloatRtk,
                            6 => FixType::Estimated,
                            7 => FixType::Manual,
                            8 => FixType::Simulation,
                            _ => FixType::Invalid,
                        }
                    }
                    Err(_) => FixType::Invalid,
                }
            }
        }

    }
}

lazy_static! {
    static ref REGEX_CHECKSUM: Regex = {
        Regex::new(r"^\$(?P<sentence>.*)\*(?P<checksum>..)$").unwrap()
    };
    static ref REGEX_TYPE: Regex = {
        Regex::new(r"^\$\D{2}(?P<type>\D{3}).*$").unwrap()
    };
    static ref REGEX_GGA_HEAD: Regex = {
          Regex::new(r"^\$\D\DGGA").unwrap()
    };
    static ref REGEX_GSV: Regex = {
        Regex::new(r"^\$(?P<type>\D\D)GSV,(?P<number>\d+),(?P<index>\d+),(?P<sat_num>\d+),(?P<sats>.*)\*\d\d$").unwrap()
    };
    static ref REGEX_RMC_HEAD: Regex = {
        Regex::new(r"^\$\D\DRMC").unwrap()
    };
}


#[test]
fn test_fix_type() {
    assert_eq!(FixType::from(""), FixType::Invalid);
    assert_eq!(FixType::from("0"), FixType::Invalid);
    assert_eq!(FixType::from("1"), FixType::Gps);
    assert_eq!(FixType::from("2"), FixType::DGps);
    assert_eq!(FixType::from("3"), FixType::Pps);
    assert_eq!(FixType::from("4"), FixType::Rtk);
    assert_eq!(FixType::from("5"), FixType::FloatRtk);
    assert_eq!(FixType::from("6"), FixType::Estimated);
    assert_eq!(FixType::from("7"), FixType::Manual);
    assert_eq!(FixType::from("8"), FixType::Simulation);
}

#[test]
fn test_parse_numeric() {
    assert_eq!(parse_numeric::<f32>("123.1", 1.0), Ok(123.1));
    assert!(parse_numeric::<f32>("123.a", 0.0).is_err());
    assert_eq!(parse_numeric::<f32>("100.1", 2.0), Ok(200.2));
    assert_eq!(parse_numeric::<f32>("-10.0", 1.0), Ok(-10.0));
    assert_eq!(parse_numeric::<f64>("123.1", 1.0), Ok(123.1));
    assert!(parse_numeric::<f64>("123.a", 0.0).is_err());
    assert_eq!(parse_numeric::<f64>("100.1", 2.0), Ok(200.2));
    assert_eq!(parse_numeric::<f64>("-10.0", 1.0), Ok(-10.0));
    assert_eq!(parse_numeric::<i32>("0", 0), Ok(0));
    assert_eq!(parse_numeric::<i32>("-10", 1), Ok(-10));
    assert_eq!(parse_numeric::<u32>("0", 0), Ok(0));
    assert!(parse_numeric::<u32>("-1", 0).is_err());
    assert_eq!(parse_numeric::<i8>("0", 0), Ok(0));
    assert_eq!(parse_numeric::<i8>("-10", 1), Ok(-10));
    assert_eq!(parse_numeric::<u8>("0", 0), Ok(0));
    assert!(parse_numeric::<u8>("-1", 0).is_err());
}

#[test]
fn test_checksum() {
    let valid = "$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E";
    let invalid = "$GNZDA,165118.00,13,05,2016,00,00*71";
    let parse_error = "";
    assert_eq!(Nmea::checksum(valid), Ok(true));
    assert_eq!(Nmea::checksum(invalid), Ok(false));
    assert!(Nmea::checksum(parse_error).is_err());
}

#[test]
fn test_message_type() {
    let nmea = Nmea::new();
    let gga = "$GPGGA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E";
    let fail = "$GPXXX,A,1,,,,,,,,,,,,,99.99,99.99,99.99*2E";
    assert_eq!(nmea.sentence_type(gga).unwrap(), SentenceType::GGA);
    assert!(nmea.sentence_type(fail).is_err());
}

#[test]
fn test_gga_north_west() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76").unwrap();
    assert_eq!(nmea.fix_timestamp_time().unwrap(), NaiveTime::from_hms(9, 27, 50));
    assert_eq!(nmea.latitude().unwrap(), 53. + 21.6802 / 60.);
    assert_eq!(nmea.longitude().unwrap(), -(6. + 30.3372 / 60.));
    assert_eq!(nmea.fix_type().unwrap(), FixType::Gps);
    assert_eq!(nmea.fix_satellites().unwrap(), 8);
    assert_eq!(nmea.hdop().unwrap(), 1.03);
    assert_eq!(nmea.geoid_height().unwrap(), 55.2);
}

#[test]
fn test_gga_north_east() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGGA,092750.000,5321.6802,N,00630.3372,E,1,8,1.03,61.7,M,55.2,M,,*64").unwrap();
    assert_eq!(nmea.latitude().unwrap(), 53. + 21.6802 / 60.);
    assert_eq!(nmea.longitude().unwrap(), 6. + 30.3372 / 60.);
}

#[test]
fn test_gga_south_west() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGGA,092750.000,5321.6802,S,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*6B").unwrap();
    assert_eq!(nmea.latitude().unwrap(), -(53. + 21.6802 / 60.));
    assert_eq!(nmea.longitude().unwrap(), -(6. + 30.3372 / 60.));
}

#[test]
fn test_gga_south_east() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGGA,092750.000,5321.6802,S,00630.3372,E,1,8,1.03,61.7,M,55.2,M,,*79").unwrap();
    assert_eq!(nmea.latitude().unwrap(), -(53. + 21.6802 / 60.));
    assert_eq!(nmea.longitude().unwrap(), 6. + 30.3372 / 60.);
}

#[test]
fn test_gga_invalid() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGGA,092750.000,5321.6802,S,00630.3372,E,0,8,1.03,61.7,M,55.2,M,,*7B").unwrap_err();
    assert_eq!(nmea.fix_type(), None);
}

#[test]
fn test_gga_gps() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGGA,092750.000,5321.6802,S,00630.3372,E,1,8,1.03,61.7,M,55.2,M,,*79").unwrap();
    assert_eq!(NaiveTime::from_hms_milli(9, 27, 50, 0), nmea.fix_timestamp_time.unwrap());
    assert_eq!(-(53. + 21.6802 / 60.), nmea.latitude.unwrap());
    assert_eq!(6. + 30.3372 / 60., nmea.longitude.unwrap());
    assert_eq!(nmea.fix_type(), Some(FixType::Gps));
    assert_eq!(8, nmea.fix_satellites.unwrap());
    assert_eq!(1.03, nmea.hdop.unwrap());
    assert_eq!(61.7, nmea.altitude.unwrap());
    assert_eq!(55.2, nmea.geoid_height.unwrap());
}

#[test]
fn test_gga_frac_sec_time() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGGA,153059.6,5650.14446,N,03546.34238,E,0,00,,,M,,M,,*41").unwrap();
    assert_eq!(NaiveTime::from_hms_milli(15, 30, 59, 600), nmea.fix_timestamp_time.unwrap());
    nmea.parse("$GPGGA,123308.2,5521.76474,N,03731.92553,E,1,08,2.2,211.5,M,13.1,M,,*52").unwrap();
    assert_eq!(NaiveTime::from_hms_milli(12, 33, 8, 200), nmea.fix_timestamp_time.unwrap());
}

#[test]
fn test_gsv() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGSV,3,1,11,10,63,137,17,07,61,098,15,05,59,290,20,08,54,157,30*70").unwrap();
    nmea.parse("$GPGSV,3,2,11,02,39,223,19,13,28,070,17,26,23,252,,04,14,186,14*79").unwrap();
    nmea.parse("$GPGSV,3,3,11,29,09,301,24,16,09,020,,36,,,*76").unwrap();
    assert_eq!(nmea.satellites().len(), 9);

    let sat: &Satellite = &(nmea.satellites()[0]);
    assert_eq!(sat.gnss_type, GnssType::Gps);
    assert_eq!(sat.prn, 10);
    assert_eq!(sat.elevation, Some(63.0));
    assert_eq!(sat.azimuth, Some(137.0));
    assert_eq!(sat.snr, Some(17.0));
}

#[test]
fn test_gsv_order() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGSV,3,2,11,02,39,223,19,13,28,070,17,26,23,252,,04,14,186,14*79").unwrap();
    nmea.parse("$GPGSV,3,3,11,29,09,301,24,16,09,020,,36,,,*76").unwrap();
    nmea.parse("$GPGSV,3,1,11,10,63,137,17,07,61,098,15,05,59,290,20,08,54,157,30*70").unwrap();
    assert_eq!(nmea.satellites().len(), 9);

    let sat: &Satellite = &(nmea.satellites()[0]);
    assert_eq!(sat.gnss_type, GnssType::Gps);
    assert_eq!(sat.prn, 10);
    assert_eq!(sat.elevation, Some(63.0));
    assert_eq!(sat.azimuth, Some(137.0));
    assert_eq!(sat.snr, Some(17.0));
}

#[test]
fn test_gsv_two_of_three() {
    let mut nmea = Nmea::new();
    nmea.parse("$GPGSV,3,2,11,02,39,223,19,13,28,070,17,26,23,252,,04,14,186,14*79").unwrap();
    nmea.parse("$GPGSV,3,3,11,29,09,301,24,16,09,020,,36,,,*76").unwrap();
    assert_eq!(nmea.satellites().len(), 6);
}

#[test]
fn test_parse() {
    let sentences = ["$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76",
                     "$GPGSA,A,3,10,07,05,02,29,04,08,13,,,,,1.72,1.03,1.38*0A",
                     "$GPGSV,3,1,11,10,63,137,17,07,61,098,15,05,59,290,20,08,54,157,30*70",
                     "$GPGSV,3,2,11,02,39,223,19,13,28,070,17,26,23,252,,04,14,186,14*79",
                     "$GPGSV,3,3,11,29,09,301,24,16,09,020,,36,,,*76",
                     "$GPRMC,092750.000,A,5321.6802,N,00630.3372,W,0.02,31.66,280511,,,A*43"];

    let mut nmea = Nmea::new();
    for s in &sentences {
        let res = nmea.parse(s);
        if s.starts_with("$GPGSA") {
            res.unwrap_err();
        } else {
            res.unwrap();
        }
    }

    assert_eq!(nmea.latitude().unwrap(), 53. + 21.6802 / 60.);
    assert_eq!(nmea.longitude().unwrap(), -(6. + 30.3372 / 60.));
    assert_eq!(nmea.altitude().unwrap(), 61.7);
}

#[test]
fn test_parse_rmc() {
    use chrono::{Datelike, Timelike};

    let rmc_data = parse_rmc("$GPRMC,225446.33,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E,A*68").unwrap();
    let fix_time = rmc_data.fix_time.unwrap();
    assert_eq!(94, fix_time.year());
    assert_eq!(11, fix_time.month());
    assert_eq!(19, fix_time.day());
    assert_eq!(22, fix_time.hour());
    assert_eq!(54, fix_time.minute());
    assert_eq!(46, fix_time.second());
    assert_eq!(330, fix_time.nanosecond() / 1_000_000);

    println!("lat: {}", rmc_data.lat.unwrap());
    assert!((rmc_data.lat.unwrap() - (49.0 + 16.45 / 60.)).abs() < 1e-5);
    println!("lon: {}, diff {}", rmc_data.lon.unwrap(), (rmc_data.lon.unwrap() + (123.0 + 11.12 / 60.)).abs());
    assert!((rmc_data.lon.unwrap() + (123.0 + 11.12 / 60.)).abs() < 1e-5);

    assert!((rmc_data.speed_over_ground.unwrap() - 0.5).abs() < 1e-5);
    assert!((rmc_data.true_course.unwrap() - 54.7).abs() < 1e-5);
}
