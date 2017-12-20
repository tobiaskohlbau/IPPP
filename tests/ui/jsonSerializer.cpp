//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <ippp/ui/FileWriterReader.h>
#include <ippp/ui/JsonSerializer.h>

using namespace ippp;

DEFINE_string(assetsDir, "../../assets", "assets directory");

template <unsigned int dim>
void testSerialization() {
    std::vector<Vector<dim>> vectors;
    for (size_t i = 0; i < 10; ++i) {
        double value = -1.23456789;
        Vector<dim> vec;
        for (unsigned int j = 0; j < dim; ++j, ++value)
            vec[j] = value;

        vectors.push_back(vec);
    }
    auto serializedData = jsonSerializer::serialize<dim>(vectors);
    auto stringData = jsonSerializer::serialize(serializedData);
    auto data = ui::load(FLAGS_assetsDir + "/tests/jsonSerializationTestDim" + std::to_string(dim) + ".json");

    EXPECT_TRUE(!stringData.compare(data));
}

TEST(JSONSERIALIZER, serialize) {
    testSerialization<2>();
    testSerialization<3>();
    testSerialization<4>();
    testSerialization<5>();
    testSerialization<6>();
    testSerialization<7>();
    testSerialization<8>();
    testSerialization<9>();
}

template <unsigned int dim>
void testDeserialization() {
    std::string serializedData = ui::load(FLAGS_assetsDir + "/tests/jsonSerializationTestDim" + std::to_string(dim) + ".json");
    nlohmann::json json;
    std::istringstream iss(serializedData);
    iss >> json;
    std::vector<Vector<dim>> vectors = jsonSerializer::deserializeVectors<dim>(json);
    for (auto &vec : vectors) {
        double value = -1.23456789;
        for (unsigned int j = 0; j < dim; ++j, ++value)
            EXPECT_NEAR(vec[j], value, 0.000001);
    }
}

TEST(JSONSERIALIZER, deserialize) {
    testDeserialization<2>();
    testDeserialization<3>();
    testDeserialization<4>();
    testDeserialization<5>();
    testDeserialization<6>();
    testDeserialization<7>();
    testDeserialization<8>();
    testDeserialization<9>();
}
