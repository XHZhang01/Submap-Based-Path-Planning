/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>
#include <se/common/str_utils.hpp>



const std::vector<std::string> s =
    {"42", "-2", "12.42", ".1", "-3.7", "-0.5f", "0-1", "1.3.", "0xD8", "7A", "abc.", ""};



TEST(StrUtils, isInt)
{
    // Create the expected outputs
    const std::vector<bool> r = {
        true, true, false, false, false, false, false, false, true, false, false, false};
    const std::vector<bool> r_nonneg = {
        true, false, false, false, false, false, false, false, true, false, false, false};
    // Test both modes of operation
    for (size_t i = 0; i < s.size(); ++i) {
        EXPECT_EQ(se::str_utils::is_int(s[i]), r[i]);
        EXPECT_EQ(se::str_utils::is_int(s[i], false), r_nonneg[i]);
    }
}



TEST(StrUtils, isFloat)
{
    // Create the expected outputs
    const std::vector<bool> r = {
        true, true, true, true, true, false, false, false, true, false, false, false};
    const std::vector<bool> r_nonneg = {
        true, false, true, true, false, false, false, false, true, false, false, false};
    // Test both modes of operation
    for (size_t i = 0; i < s.size(); ++i) {
        EXPECT_EQ(se::str_utils::is_float(s[i]), r[i]);
        EXPECT_EQ(se::str_utils::is_float(s[i], false), r_nonneg[i]);
    }
}



TEST(StrUtils, removePrefix)
{
    // Create the expected outputs
    std::vector<std::string> s = {"foo", "foo", "foofoo"};
    const std::vector<std::string> s_np = {"foo", "", "foo"};
    const std::vector<std::string> prefix = {"lol", "foo", "foo"};
    // Remove the prefixes
    for (size_t i = 0; i < s.size(); ++i) {
        se::str_utils::remove_prefix(s[i], prefix[i]);
        EXPECT_EQ(s[i], s_np[i]);
    }
}



TEST(StrUtils, removeSuffix)
{
    // Create the expected outputs
    std::vector<std::string> s = {"foo", "foo", "foofoo"};
    const std::vector<std::string> s_np = {"foo", "", "foo"};
    const std::vector<std::string> suffix = {"lol", "foo", "foo"};
    // Remove the suffixes
    for (size_t i = 0; i < s.size(); ++i) {
        se::str_utils::remove_suffix(s[i], suffix[i]);
        EXPECT_EQ(s[i], s_np[i]);
    }
}



TEST(StrUtils, resolveRelativePath)
{
    const std::string base_dir = "/baz";
    const std::vector<std::string> relative = {"", ".", "foo", "foo/bar", "/foo"};
    const std::vector<std::string> resolved_desired = {
        "/baz/", "/baz/.", "/baz/foo", "/baz/foo/bar", "/foo"};
    for (size_t i = 0; i < relative.size(); ++i) {
        const std::string resolved = se::str_utils::resolve_relative_path(relative[i], base_dir);
        EXPECT_EQ(resolved, resolved_desired[i]);
    }
}
