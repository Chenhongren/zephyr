#!/usr/bin/env python3
# vim: set syntax=python ts=4 :
#
# Copyright (c) 2018 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
import os
import contextlib
import mmap
import sys
import re
import subprocess
import glob
import logging
import json
import collections
from typing import List
from collections import OrderedDict
from itertools import islice

try:
    from anytree import RenderTree, Node, find
except ImportError:
    print("Install the anytree module to use the --test-tree option")

from twister.testsuite import TestSuite
from twister.error import TwisterRuntimeError
from twister.platform import Platform
from twister.config_parser import TwisterConfigParser
from twister.testinstance import TestInstance


from zephyr_module import west_projects, parse_modules

try:
    # Use the C LibYAML parser if available, rather than the Python parser.
    # It's much faster.
    from yaml import CSafeLoader as SafeLoader
    from yaml import CDumper as Dumper
except ImportError:
    from yaml import SafeLoader, Dumper

ZEPHYR_BASE = os.getenv("ZEPHYR_BASE")
if not ZEPHYR_BASE:
    sys.exit("$ZEPHYR_BASE environment variable undefined")

# This is needed to load edt.pickle files.
sys.path.insert(0, os.path.join(ZEPHYR_BASE, "scripts", "dts",
                                "python-devicetree", "src"))
from devicetree import edtlib  # pylint: disable=unused-import

from twister.enviornment import TwisterEnv, canonical_zephyr_base

sys.path.insert(0, os.path.join(ZEPHYR_BASE, "scripts/"))

import scl


logger = logging.getLogger('twister')
logger.setLevel(logging.DEBUG)


class ScanPathResult:
    """Result of the TestSuite.scan_path function call.

    Attributes:
        matches                          A list of test cases
        warnings                         A string containing one or more
                                         warnings to display
        has_registered_test_suites       Whether or not the path contained any
                                         calls to the ztest_register_test_suite
                                         macro.
        has_run_registered_test_suites   Whether or not the path contained at
                                         least one call to
                                         ztest_run_registered_test_suites.
        has_test_main                    Whether or not the path contains a
                                         definition of test_main(void)
        ztest_suite_names                Names of found ztest suites
    """
    def __init__(self,
                 matches: List[str] = None,
                 warnings: str = None,
                 has_registered_test_suites: bool = False,
                 has_run_registered_test_suites: bool = False,
                 has_test_main: bool = False,
                 ztest_suite_names: List[str] = []):
        self.matches = matches
        self.warnings = warnings
        self.has_registered_test_suites = has_registered_test_suites
        self.has_run_registered_test_suites = has_run_registered_test_suites
        self.has_test_main = has_test_main
        self.ztest_suite_names = ztest_suite_names

    def __eq__(self, other):
        if not isinstance(other, ScanPathResult):
            return False
        return (sorted(self.matches) == sorted(other.matches) and
                self.warnings == other.warnings and
                (self.has_registered_test_suites ==
                 other.has_registered_test_suites) and
                (self.has_run_registered_test_suites ==
                 other.has_run_registered_test_suites) and
                self.has_test_main == other.has_test_main and
                (sorted(self.ztest_suite_names) ==
                 sorted(other.ztest_suite_names)))

class Filters:
    # filters provided on command line by the user/tester
    CMD_LINE = 'command line filter'
    # filters in the testsuite yaml definition
    TESTSUITE = 'testsuite filter'
    # filters realted to platform definition
    PLATFORM = 'Platform related filter'
    # in case a testcase was quarantined.
    QUARENTINE = 'Quarantine filter'


class TestPlan:
    config_re = re.compile('(CONFIG_[A-Za-z0-9_]+)[=]\"?([^\"]*)\"?$')
    dt_re = re.compile('([A-Za-z0-9_]+)[=]\"?([^\"]*)\"?$')

    ts_schema = scl.yaml_load(
        os.path.join(ZEPHYR_BASE,
                     "scripts", "schemas", "twister", "testsuite-schema.yaml"))
    quarantine_schema = scl.yaml_load(
        os.path.join(ZEPHYR_BASE,
                     "scripts", "schemas", "twister", "quarantine-schema.yaml"))

    testsuite_valid_keys = {"tags": {"type": "set", "required": False},
                       "type": {"type": "str", "default": "integration"},
                       "extra_args": {"type": "list"},
                       "extra_configs": {"type": "list"},
                       "build_only": {"type": "bool", "default": False},
                       "build_on_all": {"type": "bool", "default": False},
                       "skip": {"type": "bool", "default": False},
                       "slow": {"type": "bool", "default": False},
                       "timeout": {"type": "int", "default": 60},
                       "min_ram": {"type": "int", "default": 8},
                       "modules": {"type": "list", "default": []},
                       "depends_on": {"type": "set"},
                       "min_flash": {"type": "int", "default": 32},
                       "arch_allow": {"type": "set"},
                       "arch_exclude": {"type": "set"},
                       "extra_sections": {"type": "list", "default": []},
                       "integration_platforms": {"type": "list", "default": []},
                       "testcases": {"type": "list", "default": []},
                       "platform_type": {"type": "list", "default": []},
                       "platform_exclude": {"type": "set"},
                       "platform_allow": {"type": "set"},
                       "toolchain_exclude": {"type": "set"},
                       "toolchain_allow": {"type": "set"},
                       "filter": {"type": "str"},
                       "harness": {"type": "str", "default": "test"},
                       "harness_config": {"type": "map", "default": {}},
                       "seed": {"type": "int", "default": 0}
                       }

    SAMPLE_FILENAME = 'sample.yaml'
    TESTSUITE_FILENAME = 'testcase.yaml'

    def __init__(self, env=None):

        self.options = env.options
        self.env = env

        # Keep track of which test cases we've filtered out and why
        self.testsuites = {}
        self.quarantine = {}
        self.platforms = []
        self.platform_names = []
        self.selected_platforms = []
        self.filtered_platforms = []
        self.default_platforms = []
        self.load_errors = 0
        self.instances = dict()
        self.warnings = 0

        self.hwm = env.hwm
        # used during creating shorter build paths
        self.link_dir_counter = 0
        self.modules = []

        self.run_individual_testsuite = []

    def find_subtests(self):
        sub_tests = self.options.sub_test
        if sub_tests:
            for subtest in sub_tests:
                _subtests = self.get_testcase(subtest)
                for _subtest in _subtests:
                    self.run_individual_testsuite.append(_subtest.name)

            if self.run_individual_testsuite:
                logger.info("Running the following tests:")
                for test in self.run_individual_testsuite:
                    print(" - {}".format(test))
            else:
                raise TwisterRuntimeError("Tests not found")

    def discover(self):
        self.handle_modules()
        if self.options.test:
            self.run_individual_testsuite = self.options.test

        num = self.add_testsuites(testsuite_filter=self.run_individual_testsuite)
        if num == 0:
            raise TwisterRuntimeError("No test cases found at the specified location...")

        self.find_subtests()
        self.add_configurations()

        if self.load_errors:
            raise TwisterRuntimeError("Errors while loading configurations")

        # handle quarantine
        ql = self.options.quarantine_list
        if ql:
            self.load_quarantine(ql)

        qv = self.options.quarantine_verify
        if qv:
            if not ql:
                logger.error("No quarantine list given to be verified")
                raise TwisterRuntimeError("No quarantine list given to be verified")


    def load(self):

        if self.options.report_suffix:
            last_run = os.path.join(self.options.outdir, "twister_{}.json".format(self.options.report_suffix))
        else:
            last_run = os.path.join(self.options.outdir, "twister.json")

        if self.options.only_failed:
            self.load_from_file(last_run)
            self.selected_platforms = set(p.platform.name for p in self.instances.values())
        elif self.options.load_tests:
            self.load_from_file(self.options.load_tests)
            self.selected_platforms = set(p.platform.name for p in self.instances.values())
        elif self.options.test_only:
            # Get list of connected hardware and filter tests to only be run on connected hardware
            # in cases where no platform was specified when running the tests.
            # If the platform does not exist in the hardware map, just skip it.
            connected_list = []
            if not self.options.platform:
                for connected in self.hwm.duts:
                    if connected['connected']:
                        connected_list.append(connected['platform'])

            self.load_from_file(last_run, filter_platform=connected_list)
            self.selected_platforms = set(p.platform.name for p in self.instances.values())
        else:
            self.apply_filters()

        if self.options.subset:
            s =  self.options.subset
            try:
                subset, sets = (int(x) for x in s.split("/"))
            except ValueError as e:
                raise TwisterRuntimeError("Bad subset value.")

            if subset > sets:
                raise TwisterRuntimeError("subset should not exceed the total number of sets")

            if int(subset) > 0 and int(sets) >= int(subset):
                logger.info("Running only a subset: %s/%s" % (subset, sets))
            else:
                raise TwisterRuntimeError(f"You have provided a wrong subset value: {self.options.subset}.")

            self.generate_subset(subset, int(sets))

    def generate_subset(self, subset, sets):
        # Test instances are sorted depending on the context. For CI runs
        # the execution order is: "plat1-testA, plat1-testB, ...,
        # plat1-testZ, plat2-testA, ...". For hardware tests
        # (device_testing), were multiple physical platforms can run the tests
        # in parallel, it is more efficient to run in the order:
        # "plat1-testA, plat2-testA, ..., plat1-testB, plat2-testB, ..."
        if self.options.device_testing:
            self.instances = OrderedDict(sorted(self.instances.items(),
                                key=lambda x: x[0][x[0].find("/") + 1:]))
        else:
            self.instances = OrderedDict(sorted(self.instances.items()))

        # Do calculation based on what is actually going to be run and evaluated
        # at runtime, ignore the cases we already know going to be skipped.
        # This fixes an issue where some sets would get majority of skips and
        # basically run nothing beside filtering.
        to_run = {k : v for k,v in self.instances.items() if v.status is None}
        total = len(to_run)
        per_set = int(total / sets)
        num_extra_sets = total - (per_set * sets)

        # Try and be more fair for rounding error with integer division
        # so the last subset doesn't get overloaded, we add 1 extra to
        # subsets 1..num_extra_sets.
        if subset <= num_extra_sets:
            start = (subset - 1) * (per_set + 1)
            end = start + per_set + 1
        else:
            base = num_extra_sets * (per_set + 1)
            start = ((subset - num_extra_sets - 1) * per_set) + base
            end = start + per_set

        sliced_instances = islice(to_run.items(), start, end)
        skipped = {k : v for k,v in self.instances.items() if v.status == 'skipped'}
        errors = {k : v for k,v in self.instances.items() if v.status == 'error'}
        self.instances = OrderedDict(sliced_instances)
        if subset == 1:
            # add all pre-filtered tests that are skipped or got error status
            # to the first set to allow for better distribution among all sets.
            self.instances.update(skipped)
            self.instances.update(errors)


    def handle_modules(self):
        # get all enabled west projects
        west_proj = west_projects()
        modules_meta = parse_modules(ZEPHYR_BASE,
                                    [p.posixpath for p in west_proj['projects']]
                                    if west_proj else None, None)
        self.modules = [module.meta.get('name') for module in modules_meta]


    def report(self):
        if self.options.list_test_duplicates:
            self.report_duplicates()
            return 0
        elif self.options.test_tree:
            self.report_test_tree()
            return 0
        elif self.options.list_tests:
            self.report_test_list()
            return 0
        elif self.options.list_tags:
            self.report_tag_list()
            return 0

        return 1

    def report_duplicates(self):
        all_tests = self.get_all_tests()

        dupes = [item for item, count in collections.Counter(all_tests).items() if count > 1]
        if dupes:
            print("Tests with duplicate identifiers:")
            for dupe in dupes:
                print("- {}".format(dupe))
                for dc in self.get_testsuite(dupe):
                    print("  - {}".format(dc))
        else:
            print("No duplicates found.")

    def report_tag_list(self):
        tags = set()
        for _, tc in self.testsuites.items():
            tags = tags.union(tc.tags)

        for t in tags:
            print("- {}".format(t))

    def report_test_tree(self):
        all_tests = self.get_all_tests()

        testsuite = Node("Testsuite")
        samples = Node("Samples", parent=testsuite)
        tests = Node("Tests", parent=testsuite)

        for test in sorted(all_tests):
            if test.startswith("sample."):
                sec = test.split(".")
                area = find(samples, lambda node: node.name == sec[1] and node.parent == samples)
                if not area:
                    area = Node(sec[1], parent=samples)

                t = Node(test, parent=area)
            else:
                sec = test.split(".")
                area = find(tests, lambda node: node.name == sec[0] and node.parent == tests)
                if not area:
                    area = Node(sec[0], parent=tests)

                if area and len(sec) > 2:
                    subarea = find(area, lambda node: node.name == sec[1] and node.parent == area)
                    if not subarea:
                        subarea = Node(sec[1], parent=area)
                    t = Node(test, parent=subarea)

        for pre, _, node in RenderTree(testsuite):
            print("%s%s" % (pre, node.name))

    def report_test_list(self):
        cnt = 0
        all_tests = self.get_all_tests()

        for test in sorted(all_tests):
            cnt = cnt + 1
            print(" - {}".format(test))

        print("{} total.".format(cnt))


    def report_excluded_tests(self):
        all_tests = self.get_all_tests()
        to_be_run = set()
        for i, p in self.instances.items():
            to_be_run.update(p.testsuite.cases)

        if all_tests - to_be_run:
            print("Tests that never build or run:")
            for not_run in all_tests - to_be_run:
                print("- {}".format(not_run))
        return

    def report_platform_tests(self, platforms=[]):
        if len(platforms) > 1:
            raise TwisterRuntimeError("When exporting tests, only one platform "
                                      "should be specified.")

        for p in platforms:
            inst = self.get_platform_instances(p)
            count = 0
            for i in inst.values():
                for c in i.testsuite.cases:
                    print(f"- {c}")
                    count += 1
            print(f"Tests found: {count}")

        return

    def get_platform_instances(self, platform):
        filtered_dict = {k:v for k,v in self.instances.items() if k.startswith(platform + os.sep)}
        return filtered_dict

    def config(self):
        logger.info("coverage platform: {}".format(self.coverage_platform))

    # Debug Functions
    @staticmethod
    def info(what):
        sys.stdout.write(what + "\n")
        sys.stdout.flush()


    def add_configurations(self):
        for board_root in self.env.board_roots:
            board_root = os.path.abspath(board_root)
            logger.debug("Reading platform configuration files under %s..." %
                         board_root)

            for file in glob.glob(os.path.join(board_root, "*", "*", "*.yaml")):
                try:
                    platform = Platform()
                    platform.load(file)
                    if platform.name in [p.name for p in self.platforms]:
                        logger.error(f"Duplicate platform {platform.name} in {file}")
                        raise Exception(f"Duplicate platform identifier {platform.name} found")
                    if platform.twister:
                        self.platforms.append(platform)
                        if platform.default:
                            self.default_platforms.append(platform.name)

                except RuntimeError as e:
                    logger.error("E: %s: can't load: %s" % (file, e))
                    self.load_errors += 1

        self.platform_names = [p.name for p in self.platforms]

    def get_all_tests(self):
        testcases = []
        for _, ts in self.testsuites.items():
            for case in ts.testcases:
                testcases.append(case)

        return testcases

    def add_testsuites(self, testsuite_filter=[]):
        for root in self.env.test_roots:
            root = os.path.abspath(root)

            logger.debug("Reading test case configuration files under %s..." % root)

            for dirpath, _, filenames in os.walk(root, topdown=True):
                if self.SAMPLE_FILENAME in filenames:
                    filename = self.SAMPLE_FILENAME
                elif self.TESTSUITE_FILENAME in filenames:
                    filename = self.TESTSUITE_FILENAME
                else:
                    continue

                logger.debug("Found possible test case in " + dirpath)

                ts_path = os.path.join(dirpath, filename)

                try:
                    parsed_data = TwisterConfigParser(ts_path, self.ts_schema)
                    parsed_data.load()

                    ts_path = os.path.dirname(ts_path)
                    workdir = os.path.relpath(ts_path, root)

                    subcases, ztest_suite_names = self.scan_path(ts_path)

                    for name in parsed_data.scenarios.keys():
                        ts = TestSuite(root, workdir, name)

                        ts_dict = parsed_data.get_scenario(name, self.testsuite_valid_keys)

                        ts.source_dir = ts_path
                        ts.yamlfile = ts_path

                        ts.type = ts_dict["type"]
                        ts.tags = ts_dict["tags"]
                        ts.extra_args = ts_dict["extra_args"]
                        ts.extra_configs = ts_dict["extra_configs"]
                        ts.arch_allow = ts_dict["arch_allow"]
                        ts.arch_exclude = ts_dict["arch_exclude"]
                        ts.skip = ts_dict["skip"]
                        ts.platform_exclude = ts_dict["platform_exclude"]
                        ts.platform_allow = ts_dict["platform_allow"]
                        ts.platform_type = ts_dict["platform_type"]
                        ts.toolchain_exclude = ts_dict["toolchain_exclude"]
                        ts.toolchain_allow = ts_dict["toolchain_allow"]
                        ts.ts_filter = ts_dict["filter"]
                        ts.timeout = ts_dict["timeout"]
                        ts.harness = ts_dict["harness"]
                        ts.harness_config = ts_dict["harness_config"]
                        if ts.harness == 'console' and not ts.harness_config:
                            raise Exception('Harness config error: console harness defined without a configuration.')
                        ts.build_only = ts_dict["build_only"]
                        ts.build_on_all = ts_dict["build_on_all"]
                        ts.slow = ts_dict["slow"]
                        ts.min_ram = ts_dict["min_ram"]
                        ts.modules = ts_dict["modules"]
                        ts.depends_on = ts_dict["depends_on"]
                        ts.min_flash = ts_dict["min_flash"]
                        ts.extra_sections = ts_dict["extra_sections"]
                        ts.integration_platforms = ts_dict["integration_platforms"]
                        ts.seed = ts_dict["seed"]

                        testcases = ts_dict.get("testcases", [])
                        if testcases:
                            for tc in testcases:
                                ts.add_testcase(name=f"{name}.{tc}")
                        else:
                            # only add each testcase once
                            for sub in set(subcases):
                                name = "{}.{}".format(ts.id, sub)
                                ts.add_testcase(name)

                            if not subcases:
                                ts.add_testcase(ts.id, freeform=True)

                        ts.ztest_suite_names = ztest_suite_names

                        if testsuite_filter:
                            if ts.name and ts.name in testsuite_filter:
                                self.testsuites[ts.name] = ts
                        else:
                            self.testsuites[ts.name] = ts

                except Exception as e:
                    logger.error("%s: can't load (skipping): %s" % (ts_path, e))
                    self.load_errors += 1
        return len(self.testsuites)



    def scan_file(self, inf_name):
        regular_suite_regex = re.compile(
            # do not match until end-of-line, otherwise we won't allow
            # stc_regex below to catch the ones that are declared in the same
            # line--as we only search starting the end of this match
            br"^\s*ztest_test_suite\(\s*(?P<suite_name>[a-zA-Z0-9_]+)\s*,",
            re.MULTILINE)
        registered_suite_regex = re.compile(
            br"^\s*ztest_register_test_suite"
            br"\(\s*(?P<suite_name>[a-zA-Z0-9_]+)\s*,",
            re.MULTILINE)
        new_suite_regex = re.compile(
            br"^\s*ZTEST_SUITE\(\s*(?P<suite_name>[a-zA-Z0-9_]+)\s*,",
            re.MULTILINE)
        # Checks if the file contains a definition of "void test_main(void)"
        # Since ztest provides a plain test_main implementation it is OK to:
        # 1. register test suites and not call the run function iff the test
        #    doesn't have a custom test_main.
        # 2. register test suites and a custom test_main definition iff the test
        #    also calls ztest_run_registered_test_suites.
        test_main_regex = re.compile(
            br"^\s*void\s+test_main\(void\)",
            re.MULTILINE)
        registered_suite_run_regex = re.compile(
            br"^\s*ztest_run_registered_test_suites\("
            br"(\*+|&)?(?P<state_identifier>[a-zA-Z0-9_]+)\)",
            re.MULTILINE)

        warnings = None
        has_registered_test_suites = False
        has_run_registered_test_suites = False
        has_test_main = False

        with open(inf_name) as inf:
            if os.name == 'nt':
                mmap_args = {'fileno': inf.fileno(), 'length': 0, 'access': mmap.ACCESS_READ}
            else:
                mmap_args = {'fileno': inf.fileno(), 'length': 0, 'flags': mmap.MAP_PRIVATE, 'prot': mmap.PROT_READ,
                             'offset': 0}

            with contextlib.closing(mmap.mmap(**mmap_args)) as main_c:
                regular_suite_regex_matches = \
                    [m for m in regular_suite_regex.finditer(main_c)]
                registered_suite_regex_matches = \
                    [m for m in registered_suite_regex.finditer(main_c)]
                new_suite_regex_matches = \
                    [m for m in new_suite_regex.finditer(main_c)]

                if registered_suite_regex_matches:
                    has_registered_test_suites = True
                if registered_suite_run_regex.search(main_c):
                    has_run_registered_test_suites = True
                if test_main_regex.search(main_c):
                    has_test_main = True

                if regular_suite_regex_matches:
                    ztest_suite_names = \
                        self._extract_ztest_suite_names(regular_suite_regex_matches)
                    testcase_names, warnings = \
                        self._find_regular_ztest_testcases(main_c, regular_suite_regex_matches, has_registered_test_suites)
                elif registered_suite_regex_matches:
                    ztest_suite_names = \
                        self._extract_ztest_suite_names(registered_suite_regex_matches)
                    testcase_names, warnings = \
                        self._find_regular_ztest_testcases(main_c, registered_suite_regex_matches, has_registered_test_suites)
                elif new_suite_regex_matches:
                    ztest_suite_names = \
                        self._extract_ztest_suite_names(new_suite_regex_matches)
                    testcase_names, warnings = \
                        self._find_new_ztest_testcases(main_c)
                else:
                    # can't find ztest_test_suite, maybe a client, because
                    # it includes ztest.h
                    ztest_suite_names = []
                    testcase_names, warnings = None, None

                return ScanPathResult(
                    matches=testcase_names,
                    warnings=warnings,
                    has_registered_test_suites=has_registered_test_suites,
                    has_run_registered_test_suites=has_run_registered_test_suites,
                    has_test_main=has_test_main,
                    ztest_suite_names=ztest_suite_names)

    @staticmethod
    def _extract_ztest_suite_names(suite_regex_matches):
        ztest_suite_names = \
            [m.group("suite_name") for m in suite_regex_matches]
        ztest_suite_names = \
            [name.decode("UTF-8") for name in ztest_suite_names]
        return ztest_suite_names

    def _find_regular_ztest_testcases(self, search_area, suite_regex_matches, is_registered_test_suite):
        """
        Find regular ztest testcases like "ztest_unit_test" or similar. Return
        testcases' names and eventually found warnings.
        """
        testcase_regex = re.compile(
            br"""^\s*  # empty space at the beginning is ok
            # catch the case where it is declared in the same sentence, e.g:
            #
            # ztest_test_suite(mutex_complex, ztest_user_unit_test(TESTNAME));
            # ztest_register_test_suite(n, p, ztest_user_unit_test(TESTNAME),
            (?:ztest_
              (?:test_suite\(|register_test_suite\([a-zA-Z0-9_]+\s*,\s*)
              [a-zA-Z0-9_]+\s*,\s*
            )?
            # Catch ztest[_user]_unit_test-[_setup_teardown](TESTNAME)
            ztest_(?:1cpu_)?(?:user_)?unit_test(?:_setup_teardown)?
            # Consume the argument that becomes the extra testcase
            \(\s*(?P<testcase_name>[a-zA-Z0-9_]+)
            # _setup_teardown() variant has two extra arguments that we ignore
            (?:\s*,\s*[a-zA-Z0-9_]+\s*,\s*[a-zA-Z0-9_]+)?
            \s*\)""",
            # We don't check how it finishes; we don't care
            re.MULTILINE | re.VERBOSE)
        achtung_regex = re.compile(
            br"(#ifdef|#endif)",
            re.MULTILINE)

        search_start, search_end = \
            self._get_search_area_boundary(search_area, suite_regex_matches, is_registered_test_suite)
        limited_search_area = search_area[search_start:search_end]
        testcase_names, warnings = \
            self._find_ztest_testcases(limited_search_area, testcase_regex)

        achtung_matches = re.findall(achtung_regex, limited_search_area)
        if achtung_matches and warnings is None:
            achtung = ", ".join(sorted({match.decode() for match in achtung_matches},reverse = True))
            warnings = f"found invalid {achtung} in ztest_test_suite()"

        return testcase_names, warnings

    @staticmethod
    def _get_search_area_boundary(search_area, suite_regex_matches, is_registered_test_suite):
        """
        Get search area boundary based on "ztest_test_suite(...)",
        "ztest_register_test_suite(...)" or "ztest_run_test_suite(...)"
        functions occurrence.
        """
        suite_run_regex = re.compile(
            br"^\s*ztest_run_test_suite\((?P<suite_name>[a-zA-Z0-9_]+)\)",
            re.MULTILINE)

        search_start = suite_regex_matches[0].end()

        suite_run_match = suite_run_regex.search(search_area)
        if suite_run_match:
            search_end = suite_run_match.start()
        elif not suite_run_match and not is_registered_test_suite:
            raise ValueError("can't find ztest_run_test_suite")
        else:
            search_end = re.compile(br"\);", re.MULTILINE) \
                .search(search_area, search_start) \
                .end()

        return search_start, search_end

    def _find_new_ztest_testcases(self, search_area):
        """
        Find regular ztest testcases like "ZTEST" or "ZTEST_F". Return
        testcases' names and eventually found warnings.
        """
        testcase_regex = re.compile(
            br"^\s*(?:ZTEST|ZTEST_F|ZTEST_USER|ZTEST_USER_F)\(\s*(?P<suite_name>[a-zA-Z0-9_]+)\s*,"
            br"\s*(?P<testcase_name>[a-zA-Z0-9_]+)\s*",
            re.MULTILINE)

        return self._find_ztest_testcases(search_area, testcase_regex)

    @staticmethod
    def _find_ztest_testcases(search_area, testcase_regex):
        """
        Parse search area and try to find testcases defined in testcase_regex
        argument. Return testcase names and eventually found warnings.
        """
        testcase_regex_matches = \
            [m for m in testcase_regex.finditer(search_area)]
        testcase_names = \
            [m.group("testcase_name") for m in testcase_regex_matches]
        testcase_names = [name.decode("UTF-8") for name in testcase_names]
        warnings = None
        for testcase_name in testcase_names:
            if not testcase_name.startswith("test_"):
                warnings = "Found a test that does not start with test_"
        testcase_names = \
            [tc_name.replace("test_", "", 1) for tc_name in testcase_names]

        return testcase_names, warnings

    def scan_path(self, path):
        subcases = []
        has_registered_test_suites = False
        has_run_registered_test_suites = False
        has_test_main = False
        ztest_suite_names = []

        src_dir_path = self._find_src_dir_path(path)
        for filename in glob.glob(os.path.join(src_dir_path, "*.c*")):
            try:
                result: ScanPathResult = self.scan_file(filename)
                if result.warnings:
                    logger.error("%s: %s" % (filename, result.warnings))
                    raise TwisterRuntimeError(
                        "%s: %s" % (filename, result.warnings))
                if result.matches:
                    subcases += result.matches
                if result.has_registered_test_suites:
                    has_registered_test_suites = True
                if result.has_run_registered_test_suites:
                    has_run_registered_test_suites = True
                if result.has_test_main:
                    has_test_main = True
                if result.ztest_suite_names:
                    ztest_suite_names += result.ztest_suite_names

            except ValueError as e:
                logger.error("%s: can't find: %s" % (filename, e))

        for filename in glob.glob(os.path.join(path, "*.c")):
            try:
                result: ScanPathResult = self.scan_file(filename)
                if result.warnings:
                    logger.error("%s: %s" % (filename, result.warnings))
                if result.matches:
                    subcases += result.matches
                if result.ztest_suite_names:
                    ztest_suite_names += result.ztest_suite_names
            except ValueError as e:
                logger.error("%s: can't find: %s" % (filename, e))

        if (has_registered_test_suites and has_test_main and
                not has_run_registered_test_suites):
            warning = \
                "Found call to 'ztest_register_test_suite()' but no "\
                "call to 'ztest_run_registered_test_suites()'"
            logger.error(warning)
            raise TwisterRuntimeError(warning)

        return subcases, ztest_suite_names

    @staticmethod
    def _find_src_dir_path(test_dir_path):
        """
        Try to find src directory with test source code. Sometimes due to the
        optimization reasons it is placed in upper directory.
        """
        src_dir_name = "src"
        src_dir_path = os.path.join(test_dir_path, src_dir_name)
        if os.path.isdir(src_dir_path):
            return src_dir_path
        src_dir_path = os.path.join(test_dir_path, "..", src_dir_name)
        if os.path.isdir(src_dir_path):
            return src_dir_path
        return ""

    def __str__(self):
        return self.name

    def get_platform(self, name):
        selected_platform = None
        for platform in self.platforms:
            if platform.name == name:
                selected_platform = platform
                break
        return selected_platform

    def load_quarantine(self, file):
        """
        Loads quarantine list from the given yaml file. Creates a dictionary
        of all tests configurations (platform + scenario: comment) that shall be
        skipped due to quarantine
        """

        # Load yaml into quarantine_yaml
        quarantine_yaml = scl.yaml_load_verify(file, self.quarantine_schema)

        # Create quarantine_list with a product of the listed
        # platforms and scenarios for each entry in quarantine yaml
        quarantine_list = []
        for quar_dict in quarantine_yaml:
            if quar_dict['platforms'][0] == "all":
                plat = self.platform_names
            else:
                plat = quar_dict['platforms']
            comment = quar_dict.get('comment', "NA")
            quarantine_list.append([{".".join([p, s]): comment}
                                   for p in plat for s in quar_dict['scenarios']])

        # Flatten the quarantine_list
        quarantine_list = [it for sublist in quarantine_list for it in sublist]
        # Change quarantine_list into a dictionary
        for d in quarantine_list:
            self.quarantine.update(d)

    def load_from_file(self, file, filter_platform=[]):
        with open(file, "r") as json_test_plan:
            jtp = json.load(json_test_plan)
            instance_list = []
            for ts in jtp.get("testsuites", []):
                logger.debug(f"loading {ts['name']}...")
                testsuite = ts["name"]

                platform = self.get_platform(ts["platform"])
                if filter_platform and platform.name not in filter_platform:
                    continue
                instance = TestInstance(self.testsuites[testsuite], platform, self.env.outdir)
                if ts.get("run_id"):
                    instance.run_id = ts.get("run_id")

                if self.options.device_testing:
                    tfilter = 'runnable'
                else:
                    tfilter = 'buildable'
                instance.run = instance.check_runnable(
                    self.options.enable_slow,
                    tfilter,
                    self.options.fixture
                )

                instance.metrics['handler_time'] = ts.get('execution_time', 0)
                instance.metrics['ram_size'] = ts.get("ram_size", 0)
                instance.metrics['rom_size']  = ts.get("rom_size",0)

                status = ts.get('status', None)
                reason = ts.get("reason", "Unknown")
                if status in ["error", "failed"]:
                    instance.status = None
                    instance.reason = None
                # test marked as passed (built only) but can run when
                # --test-only is used. Reset status to capture new results.
                elif status == 'passed' and instance.run and self.options.test_only:
                    instance.status = None
                    instance.reason = None
                else:
                    instance.status = status
                    instance.reason = reason

                for tc in ts.get('testcases', []):
                    identifier = tc['identifier']
                    tc_status = tc.get('status', None)
                    tc_reason = None
                    # we set reason only if status is valid, it might have been
                    # reset above...
                    if instance.status:
                        tc_reason = tc.get('reason')
                    if tc_status:
                        case = instance.set_case_status_by_name(identifier, tc_status, tc_reason)
                        case.duration = tc.get('execution_time', 0)
                        if tc.get('log'):
                            case.output = tc.get('log')


                instance.create_overlay(platform, self.options.enable_asan, self.options.enable_ubsan, self.options.enable_coverage, self.options.coverage_platform)
                instance_list.append(instance)
            self.add_instances(instance_list)

    def apply_filters(self, **kwargs):

        toolchain = self.env.toolchain
        platform_filter = self.options.platform
        exclude_platform = self.options.exclude_platform
        testsuite_filter = self.run_individual_testsuite
        arch_filter = self.options.arch
        tag_filter = self.options.tag
        exclude_tag = self.options.exclude_tag
        all_filter = self.options.all
        runnable = (self.options.device_testing or self.options.filter == 'runnable')
        force_toolchain = self.options.force_toolchain
        force_platform = self.options.force_platform
        emu_filter = self.options.emulation_only

        logger.debug("platform filter: " + str(platform_filter))
        logger.debug("    arch_filter: " + str(arch_filter))
        logger.debug("     tag_filter: " + str(tag_filter))
        logger.debug("    exclude_tag: " + str(exclude_tag))

        default_platforms = False
        emulation_platforms = False


        if all_filter:
            logger.info("Selecting all possible platforms per test case")
            # When --all used, any --platform arguments ignored
            platform_filter = []
        elif not platform_filter and not emu_filter:
            logger.info("Selecting default platforms per test case")
            default_platforms = True
        elif emu_filter:
            logger.info("Selecting emulation platforms per test case")
            emulation_platforms = True

        if platform_filter:
            self.verify_platforms_existence(platform_filter, f"platform_filter")
            platforms = list(filter(lambda p: p.name in platform_filter, self.platforms))
        elif emu_filter:
            platforms = list(filter(lambda p: p.simulation != 'na', self.platforms))
        elif arch_filter:
            platforms = list(filter(lambda p: p.arch in arch_filter, self.platforms))
        elif default_platforms:
            platforms = list(filter(lambda p: p.default, self.platforms))
        else:
            platforms = self.platforms

        logger.info("Building initial testsuite list...")

        for ts_name, ts in self.testsuites.items():

            if ts.build_on_all and not platform_filter:
                platform_scope = self.platforms
            elif ts.integration_platforms and self.options.integration:
                self.verify_platforms_existence(
                    ts.integration_platforms, f"{ts_name} - integration_platforms")
                platform_scope = list(filter(lambda item: item.name in ts.integration_platforms, \
                                         self.platforms))
            else:
                platform_scope = platforms

            integration = self.options.integration and ts.integration_platforms

            # If there isn't any overlap between the platform_allow list and the platform_scope
            # we set the scope to the platform_allow list
            if ts.platform_allow and not platform_filter and not integration:
                self.verify_platforms_existence(
                    ts.platform_allow, f"{ts_name} - platform_allow")
                a = set(platform_scope)
                b = set(filter(lambda item: item.name in ts.platform_allow, self.platforms))
                c = a.intersection(b)
                if not c:
                    platform_scope = list(filter(lambda item: item.name in ts.platform_allow, \
                                             self.platforms))

            # list of instances per testsuite, aka configurations.
            instance_list = []
            for plat in platform_scope:
                instance = TestInstance(ts, plat, self.env.outdir)
                if runnable:
                    tfilter = 'runnable'
                else:
                    tfilter = 'buildable'

                instance.run = instance.check_runnable(
                    self.options.enable_slow,
                    tfilter,
                    self.options.fixture
                )
                if runnable and self.hwm.duts:
                    for h in self.hwm.duts:
                        if h.platform == plat.name:
                            if ts.harness_config.get('fixture') in h.fixtures:
                                instance.run = True

                if not force_platform and plat.name in exclude_platform:
                    instance.add_filter("Platform is excluded on command line.", Filters.CMD_LINE)

                if (plat.arch == "unit") != (ts.type == "unit"):
                    # Discard silently
                    continue

                if ts.modules and self.modules:
                    if not set(ts.modules).issubset(set(self.modules)):
                        instance.add_filter(f"one or more required modules not available: {','.join(ts.modules)}", Filters.TESTSUITE)

                if runnable and not instance.run:
                    instance.add_filter("Not runnable on device", Filters.PLATFORM)

                if self.options.integration and ts.integration_platforms and plat.name not in ts.integration_platforms:
                    instance.add_filter("Not part of integration platforms", Filters.TESTSUITE)

                if ts.skip:
                    instance.add_filter("Skip filter", Filters.TESTSUITE)

                if tag_filter and not ts.tags.intersection(tag_filter):
                    instance.add_filter("Command line testsuite tag filter", Filters.CMD_LINE)

                if exclude_tag and ts.tags.intersection(exclude_tag):
                    instance.add_filter("Command line testsuite exclude filter", Filters.CMD_LINE)

                if testsuite_filter and ts_name not in testsuite_filter:
                    instance.add_filter("TestSuite name filter", Filters.CMD_LINE)

                if arch_filter and plat.arch not in arch_filter:
                    instance.add_filter("Command line testsuite arch filter", Filters.CMD_LINE)

                if not force_platform:

                    if ts.arch_allow and plat.arch not in ts.arch_allow:
                        instance.add_filter("Not in test case arch allow list", Filters.TESTSUITE)

                    if ts.arch_exclude and plat.arch in ts.arch_exclude:
                        instance.add_filter("In test case arch exclude", Filters.TESTSUITE)

                    if ts.platform_exclude and plat.name in ts.platform_exclude:
                        instance.add_filter("In test case platform exclude", Filters.TESTSUITE)

                if ts.toolchain_exclude and toolchain in ts.toolchain_exclude:
                    instance.add_filter("In test case toolchain exclude", Filters.TESTSUITE)

                if platform_filter and plat.name not in platform_filter:
                    instance.add_filter("Command line platform filter", Filters.CMD_LINE)

                if ts.platform_allow and plat.name not in ts.platform_allow:
                    instance.add_filter("Not in testsuite platform allow list", Filters.TESTSUITE)

                if ts.platform_type and plat.type not in ts.platform_type:
                    instance.add_filter("Not in testsuite platform type list", Filters.TESTSUITE)

                if ts.toolchain_allow and toolchain not in ts.toolchain_allow:
                    instance.add_filter("Not in testsuite toolchain allow list", Filters.TESTSUITE)

                if not plat.env_satisfied:
                    instance.add_filter("Environment ({}) not satisfied".format(", ".join(plat.env)), Filters.PLATFORM)

                if not force_toolchain \
                        and toolchain and (toolchain not in plat.supported_toolchains) \
                        and "host" not in plat.supported_toolchains \
                        and ts.type != 'unit':
                    instance.add_filter("Not supported by the toolchain", Filters.PLATFORM)

                if plat.ram < ts.min_ram:
                    instance.add_filter("Not enough RAM", Filters.PLATFORM)

                if ts.depends_on:
                    dep_intersection = ts.depends_on.intersection(set(plat.supported))
                    if dep_intersection != set(ts.depends_on):
                        instance.add_filter("No hardware support", Filters.PLATFORM)

                if plat.flash < ts.min_flash:
                    instance.add_filter("Not enough FLASH", Filters.PLATFORM)

                if set(plat.ignore_tags) & ts.tags:
                    instance.add_filter("Excluded tags per platform (exclude_tags)", Filters.PLATFORM)

                if plat.only_tags and not set(plat.only_tags) & ts.tags:
                    instance.add_filter("Excluded tags per platform (only_tags)", Filters.PLATFORM)

                test_configuration = ".".join([instance.platform.name,
                                               instance.testsuite.id])
                # skip quarantined tests
                if test_configuration in self.quarantine and not self.options.quarantine_verify:
                    instance.add_filter(f"Quarantine: {self.quarantine[test_configuration]}", Filters.QUARENTINE)
                # run only quarantined test to verify their statuses (skip everything else)
                if self.options.quarantine_verify and test_configuration not in self.quarantine:
                    instance.add_filter("Not under quarantine", Filters.QUARENTINE)

                # if nothing stopped us until now, it means this configuration
                # needs to be added.
                instance_list.append(instance)

            # no configurations, so jump to next testsuite
            if not instance_list:
                continue

            # if twister was launched with no platform options at all, we
            # take all default platforms
            if default_platforms and not ts.build_on_all and not integration:
                if ts.platform_allow:
                    a = set(self.default_platforms)
                    b = set(ts.platform_allow)
                    c = a.intersection(b)
                    if c:
                        aa = list(filter(lambda ts: ts.platform.name in c, instance_list))
                        self.add_instances(aa)
                    else:
                        self.add_instances(instance_list)
                else:
                    instances = list(filter(lambda ts: ts.platform.default, instance_list))
                    self.add_instances(instances)
            elif integration:
                instances = list(filter(lambda item:  item.platform.name in ts.integration_platforms, instance_list))
                self.add_instances(instances)

            elif emulation_platforms:
                self.add_instances(instance_list)
                for instance in list(filter(lambda inst: not inst.platform.simulation != 'na', instance_list)):
                    instance.add_filter("Not an emulated platform", Filters.PLATFORM)
            else:
                self.add_instances(instance_list)

        for _, case in self.instances.items():
            case.create_overlay(case.platform, self.options.enable_asan, self.options.enable_ubsan, self.options.enable_coverage, self.options.coverage_platform)

        self.selected_platforms = set(p.platform.name for p in self.instances.values())

        filtered_instances = list(filter(lambda item:  item.status == "filtered", self.instances.values()))
        for filtered_instance in filtered_instances:
            # If integration mode is on all skips on integration_platforms are treated as errors.
            if self.options.integration and filtered_instance.platform.name in filtered_instance.testsuite.integration_platforms \
                and "Quarantine" not in filtered_instance.reason:
                # Do not treat this as error if filter type is command line
                filters = {t['type'] for t in filtered_instance.filters}
                if Filters.CMD_LINE in filters:
                    continue
                filtered_instance.status = "error"
                filtered_instance.reason += " but is one of the integration platforms"
                self.instances[filtered_instance.name] = filtered_instance

            filtered_instance.add_missing_case_status(filtered_instance.status)

        self.filtered_platforms = set(p.platform.name for p in self.instances.values()
                                      if p.status != "skipped" )

    def add_instances(self, instance_list):
        for instance in instance_list:
            self.instances[instance.name] = instance


    def get_testsuite(self, identifier):
        results = []
        for _, ts in self.testsuites.items():
            for case in ts.testcases:
                if case == identifier:
                    results.append(ts)
        return results

    def verify_platforms_existence(self, platform_names_to_verify, log_info=""):
        """
        Verify if platform name (passed by --platform option, or in yaml file
        as platform_allow or integration_platforms options) is correct. If not -
        log and raise error.
        """
        for platform in platform_names_to_verify:
            if platform in self.platform_names:
                break
            else:
                logger.error(f"{log_info} - unrecognized platform - {platform}")
                sys.exit(2)

    def create_build_dir_links(self):
        """
        Iterate through all no-skipped instances in suite and create links
        for each one build directories. Those links will be passed in the next
        steps to the CMake command.
        """

        links_dir_name = "twister_links"  # folder for all links
        links_dir_path = os.path.join(self.env.outdir, links_dir_name)
        if not os.path.exists(links_dir_path):
            os.mkdir(links_dir_path)

        for instance in self.instances.values():
            if instance.status != "skipped":
                self._create_build_dir_link(links_dir_path, instance)

    def _create_build_dir_link(self, links_dir_path, instance):
        """
        Create build directory with original "long" path. Next take shorter
        path and link them with original path - create link. At the end
        replace build_dir to created link. This link will be passed to CMake
        command. This action helps to limit path length which can be
        significant during building by CMake on Windows OS.
        """

        os.makedirs(instance.build_dir, exist_ok=True)

        link_name = f"test_{self.link_dir_counter}"
        link_path = os.path.join(links_dir_path, link_name)

        if os.name == "nt":  # if OS is Windows
            command = ["mklink", "/J", f"{link_path}", f"{instance.build_dir}"]
            subprocess.call(command, shell=True)
        else:  # for Linux and MAC OS
            os.symlink(instance.build_dir, link_path)

        # Here original build directory is replaced with symbolic link. It will
        # be passed to CMake command
        instance.build_dir = link_path

        self.link_dir_counter += 1
