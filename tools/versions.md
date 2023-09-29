## Gazebo versions

Summary of versions for all Gazebo libraries and their support dates.

How to read the columns:

* **Package**: Each library or application that is supported by the Open Robotics simulation team.
* **Version**: Major version for each package.
* **Status**
    * **EOL**: Reached End of Life and is no longer supported.
    * **stable**: Release is stable and may get bug fixes and backwards-compatible features.
    * **not-released**: No stable release has been made for that major version yet.
* **Packages**: Whether debian packages for a version are provided by specific repositories. Each letter refers to an Ubuntu release (i.e. B for Bionic, F for Focal, G for Groovy,...). The repositories are the following:
    * **Ubuntu**: https://packages.ubuntu.com/ (official Ubuntu archive packages)
    * **OSRF**: https://packages.osrfoundation.org/ (offical repository of the Gazebo project)
    * **ROS**: https://packages.ros.org/ (package imported from OSRF repository into ROS)
* **Gazebo Classic**: What Gazebo classic version requires as depedency that specific version (i.e. 9 for Gazebo 9).
* **Gazebo**: What Gazebo version uses that specific version (i.e. B for Blueprint).
* **Release date**: Release date for that specific version. TBD, to be decided, means that there aren't any current plans to release that version.
* **EOL date**: Date of end of support. In general, each library's version is released as part of an [Gazebo release](https://gazebosim.org/docs/all/releases) and EOLs either with that Gazebo release, or with a later one. Libraries that are planned to be supported in the upcoming Gazebo release have a TBD EOL date because they may also be included in future releases. All EOL dates may be moved to a later date, but never to an earlier date.

|Package|Version|Status|Ubuntu packages|OSRF packages|ROS packages|Gazebo Classic|Gazebo|Release date|EOL date|
|---|---|---|---|---|---|---|---|---|---|
|gz-cmake|0| EOL ❌ |B C D E F|X Z A B||||2017-10-09|2023-01-25 with Gazebo 9|
||1|EOL ❌||X B||||2018-12-12|-|
||2|stable|F|B F J|F J|11|A B C D E F|2019-01-31|2026-09 with Fortress|
||3|stable||F J|||G H|2022-09-23|TBD|
|gz-common|0|EOL ❌||T X Y Z A||||2016-07-27|-|
||1|EOL ❌|B C D E F|X Z A B||9 10||2018-01-05|2023-01-25 with Gazebo 9|
||2|EOL ❌||X B||||2019-02-11|-|
||3|stable|G H I J K L|B F|F|11|A B C D|2019-02-28|2025-01-25 with Gazebo 11|
||4|stable|J K L|B F J|F J| |E F|2020-03-31|2026-09 with Fortress|
||5|stable||F J|||G H|2022-09-22|TBD|
|gz-fuel-tools|0|EOL ❌||X Z A||||2017|-|
||1|EOL ❌|B C D E F|X A B||9 10||2018-01-25|2023-01-25 with Gazebo 9|
||2|EOL ❌||X B||||2019-01-29|2019-01-29|
||3|EOL ❌||B|||A B|2019-01-30|2020-12 with Blueprint|
||4|stable|G H I J K L|B F|F|11|C|2019-12-10|2025-01-25 with Gazebo 11|
||5|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||6|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||7|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||8|stable||F J|||G|2022-09-26|2024-11 with Garden|
||9|stable||J|||H|2022-09-26|TBD|
|gz-sim|1|EOL ❌||B|||A|2019-03-02|2019-09 with Acropolis|
||2|EOL ❌||B|||B|2019-05-21|2020-12 with Blueprint|
||3|stable||B F|F||C|2019-12-10|2024-12 with Citadel|
||4|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||5|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||6|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||7|stable||F J|||G|2022-09-27|2024-11 with Garden|
||8|stable||J|||H|2023-09-29|TBD|
|gz-gui|0|EOL ❌||B||||2019-03-06|-|
||1|EOL ❌||B|||A|2019-03-01|2019-09 with Acropolis|
||2|EOL ❌||B|||B|2019-05-21|2020-12 with Blueprint|
||3|stable||B F|F||C|2019-12-10|2024-12 with Citadel|
||4|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||5|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||6|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||7|stable||F J|||G|2022-09-27|2024-11 with Garden|
||8|stable||J|||H|2023-09-29|TBD|
|gz-launch|0|EOL ❌||B|||A|2019-03-18|2019-09 with Acropolis|
||1|EOL ❌||B|||B|2019-05-21|2020-12 with Blueprint|
||2|stable||B F|F||C|2019-12-10|2024-12 with Citadel|
||3|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||4|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||5|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||6|stable||F J|||G|2022-09-27|2024-11 with Garden|
||7|stable||J|||H|2023-09-29|TBD|
|gz-math|0|EOL ❌||P T||||2014-06-16|-|
||1|EOL ❌||||||2015-03-06|-|
||2|EOL ❌|X B C D E|P T V W X Y Z A||6 7||2015-04-17|2021-01-25 with gazebo7|
||3|EOL ❌||T X Y Z A||8||2017-01-05|2019-01-25 with gazebo8|
||4|EOL ❌|B C D E F|X Z A B||9 10||2017-12-26|2023-01-25 with Gazebo 9|
||5|EOL ❌||X B||||2018-12-12||
||6|stable|G H I J K L|B F J|F J|11|A B C D E F|2019-01-31|2026-09 with Fortress|
||7|stable||F J|||G H|2022-09-22|TBD|
|gz-msgs|0|EOL ❌||P T X Z A B||8||2014-07-14|2019-01-25 with gazebo8|
||1|EOL ❌|B C D E F|T W X Y Z A||9 10||2017-10-04|2023-01-25 with Gazebo 9|
||2|EOL ❌||X B||||2019-02-11||
||3|EOL ❌||B|||A|2019-02-27|2019-09 with Acropolis|
||4|EOL ❌||B|||B|2019-05-20|2020-12 with Blueprint|
||5|stable|G H I J K L|B F|F|11|C|2019-12-10|2025-01-25 with Gazebo 11|
||6|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||7|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||8|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||9|stable||F J|||G|2022-09-22|2024-11 with Garden|
||10|stable|J||||H|2023-09-29|TBD|
|gz-physics|1|EOL ❌||B|||A B|2019-03-01|2020-12 with Blueprint|
||2|stable||B F|F||C|2019-12-10|2024-12 with Citadel|
||3|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||4|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||5|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||6|stable||F J|||G|2022-09-21|2024-11 with Garden|
||7|stable||J|||H|2023-09-29|TBD|
|gz-plugin|1|stable|J K L|B F J|F J||A B C D E F|2019-03-01|2026-09 with Fortress|
||2|stable||F J|||G H|2022-09-22|TBD|
|gz-rendering|1|EOL ❌||B|||A|2019-02-28|2019-09 with Acropolis|
||2|EOL ❌||B|||B|2019-05-20|2020-12 with Blueprint|
||3|stable||B F|F||C|2019-12-10|2024-12 with Citadel|
||4|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||5|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||6|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||7|stable||F J|||G|2022-09-22|2024-11 with Garden|
||8|stable|J||||H|2023-09-29|TBD|
|gz-sensors|1|EOL ❌||B|||A|2019-03-01|2019-09 with Acropolis|
||2|EOL ❌||B|||B|2019-05-21|2020-12 with Blueprint|
||3|stable||B F|F||C|2019-12-10|2024-12 with Citadel|
||4|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||5|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||6|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||7|stable||F J|||G|2022-09-22|2024-11 with Garden|
||8|stable|J||||H|2023-09-29|TBD|
|gz-tools|0|EOL ❌||X A B|||A B|2018-02-22|2020-12 with Blueprint|
||1|stable|J K L|B F J|F J|11|C D E F|2019-05-21|2026-09 with Fortress|
||2|stable||F J|||G H|2022-09-22|TBD|
|gz-transport|0|EOL ❌|X|P T V W||||2014-08-12||
||1|EOL ❌||T V W X||||2016-02-05||
||2|EOL ❌||T X Y||7||2016-08-08|2021-01-25 with gazebo7|
||3|EOL ❌||T X Y Z A||8||2016-12-16|2019-01-25 with gazebo8|
||4|EOL ❌|B C D E F|X A||9 10||2018-01-25|2023-01-25 with Gazebo 9|
||5|EOL ❌||X B||||2019-02-11||
||6|EOL ❌||B|||A|2019-02-28|2019-09 with Acropolis|
||7|EOL ❌||B|||B|2019-05-20|2020-12 with Blueprint|
||8|stable|J K L|B F|F|11|C|2019-12-10|2025-01-25 with Gazebo 11|
||9|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||10|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||11|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||12|stable||F J|||G|2022-09-22|2024-11 with Garden|
||13|stable||J|||H|2023-09-29|TBD|
|gz-utils|1|stable|J K L|B F J|F J||E F|2020-03-31|2026-09 with Fortress|
||2|stable||F J|||G H|2022-09-22|TBD|
|SDFormat|1|EOL ❌||P W||2||2013-03-28|2016-01-25 with gazebo2|
||2|EOL ❌||P T V||3 4 5||2014-04-11|2017-01-25 with gazebo5|
||3|EOL ❌||T V||6||2015-07-24|2017-01-25 with gazebo6|
||4|EOL ❌|X|T V W X Y||7||2016-01-13|2021-01-25 with gazebo7|
||5|EOL ❌||T X Y Z||8||2017-01-25|2019-01-25 with gazebo8|
||6|EOL ❌|B C D E F J K L|X A B F||9 10||2018-01-25|2023-01-25 with Gazebo 9|
||7|EOL ❌||||||||
||8|EOL ❌||B|||A B|2019-03-01|2020-12 with Blueprint|
||9|stable|J K L|B F|F|11|C|2019-12-10|2025-01-25 with Gazebo 11|
||10|EOL ❌|||||D|2020-09-30|2021-12 with Dome|
||11|EOL ❌|||F||E|2020-03-31|2022-03 with Edifice|
||12|stable|J K L|B F J|F J||F|2021-09-30|2026-09 with Fortress|
||13|stable||F J|||G|2022-09-23|2024-11 with Garden|
||14|stable||J|||H|2022-09-23|TBD|
|Gazebo classic|1|EOL ❌||P||||2012-12-09|2015-07-27|
||2|EOL ❌||T||||2013-10-08|2016-01-25|
||3|EOL ❌||P T||||2014-04-11|2015-07-27|
||4|EOL ❌||P T||||2014-08-07|2016-01-25|
||5|EOL ❌||T V||||2015-01-20|2017-01-25|
||6|EOL ❌||T V||||2015-07-28|2017-01-25|
||7|EOL ❌|X|T V W X Z A||||2016-01-26|2021-01-25|
||8|EOL ❌||X Y Z ||||2017-01-26|2019-01-25|
||9|EOL ❌|B C E F|X A B F|B|||2018-01-25|2023-01-25|
||10|EOL ❌||X B||||2019-01-31|2021-01-25|
||11|stable|J|B F|F|||2020-01-30|2025-01-25|
