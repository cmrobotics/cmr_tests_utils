^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cmr_tests_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2025-03-14)
------------------
* Merge pull request `#17 <https://github.com/cmrobotics/cmr_tests_utils/issues/17>`_ from cmrobotics/fix-unit-tests
  fix: unit  tests - always lookup latest available transform
* fix: unit  tests - always lookup latest available transform
  Tranforms typically not available at now() timestamp because they are published with a past timestamp before calling lookup_transform() in all unit tests.
  Even if the transform are published periodically with BasicTfBroadcasterNodeTest node, the TF published is with the same timestamp of the past, so the queried TF is never available even with the blocking transform_tolerance\_.
* Contributors: Tanmay, Tanmay Deshmukh

1.0.0 (2023-11-28)
------------------
* Initial release
* Contributors: Aaron, Arkadiusz Nowakowski, Clara Dieudonné, Erwin Lejeune, Soma Gallai

0.0.3 (2022-06-02)
------------------
* Tag without changelog
* Contributors: Erwin Lejeune

0.0.2 (2022-05-24)
------------------
* Tag without changelog
* Contributors: Erwin Lejeune

0.0.1 (2022-05-10)
------------------
* Tag without changelog
* Contributors: Erwin Lejeune, Soma Gallai
