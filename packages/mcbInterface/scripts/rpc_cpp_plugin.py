# Copyright 2021 Jeffrey van Pernis (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python3

import sys

from google.protobuf.compiler import plugin_pb2 as plugin

def generate_code(request, response):
    for proto_file in request.proto_file:
        services = []
        for service in proto_file.service:
            methods = []
            for method in service.method:
                method_data = {}
                method_data["name"] = method.name
                method_data["input_type"] = method.input_type.replace(".", "::")
                method_data["output_type"] = method.output_type.replace(".", "::")
                methods.append(method_data)

            service_data = {}
            service_data["name"] = service.name.replace(".", "::")
            service_data["methods"] = methods
            services.append(service_data)

        if len(services) > 0:
            proto_data = {}
            proto_data["name"] = proto_file.name.split(".")[0]
            proto_data["package"] = proto_file.package
            proto_data["services"] = services

            header_file = response.file.add()
            header_file.name = proto_file.name.split(".")[0] + '.rpc.pb.h'
            header_file.content = generate_header(proto_data)

            source_file = response.file.add()
            source_file.name = proto_file.name.split(".")[0] + '.rpc.pb.cc'
            source_file.content = generate_source(proto_data)

def generate_header(proto_data):
    output = []

    output.append("// THIS FILE IS GENERATED, DO NOT EDIT!")
    output.append("")
    output.append("#ifndef %s_" % (proto_data["name"].upper()))
    output.append("#define %s_" % (proto_data["name"].upper()))
    output.append("")
    output.append("#include <string>")
    output.append("")
    output.append("#include \"Rpc.hpp\"")
    output.append("")
    output.append("#include \"%s\"" % (proto_data["name"] + ".pb.h"))
    output.append("")
    output.append("namespace %s" % proto_data["package"])
    output.append("{")
    output.append("")

    for service in proto_data["services"]:
        # Generate client
        output.append("class Client : public Rpc::Client")
        output.append("{")
        output.append("public:")
        output.append("    Client(const std::string host, uint16_t port);")
        output.append("    ~Client();")
        output.append("")

        for method in service["methods"]:
            output.append("    %s %s(const %s& request);" % (method["output_type"], method["name"], method["input_type"]))

        output.append("};")
        output.append("")

        # Generate server
        output.append("class Server : public Rpc::Server")
        output.append("{")
        output.append("public:")
        output.append("    Server(uint16_t port);")
        output.append("    ~Server();")
        output.append("")

        output.append("protected:")
        for method in service["methods"]:
            output.append("    virtual void %s(const %s& request, %s& response) = 0;" % (method["name"], method["input_type"], method["output_type"]))
        output.append("")

        output.append("private:")
        output.append("    ::google::protobuf::Message* get_request_prototype(uint8_t method);")
        output.append("    ::google::protobuf::Message* get_response_prototype(uint8_t method);")
        output.append("    void call(uint8_t method, const ::google::protobuf::Message& request, ::google::protobuf::Message& response);")

        output.append("};")

    output.append("}")

    output.append("")
    output.append("#endif // %s_" % (proto_data["name"].upper()))
    output.append("")

    return "\n".join(output)

def generate_source(proto_data):
    output = []

    output.append("// THIS FILE IS GENERATED, DO NOT EDIT!")
    output.append("#include \"%s\"" % (proto_data["name"] + ".rpc.pb.h"))
    output.append("")
    output.append("namespace %s" % proto_data["package"])
    output.append("{")
    output.append("")

    for service in proto_data["services"]:
        # Generate client
        output.append("Client::Client(const std::string host, uint16_t port) :")
        output.append("    Rpc::Client(host, port)")
        output.append("{")
        output.append("}")
        output.append("")

        output.append("Client::~Client()")
        output.append("{")
        output.append("}")
        output.append("")

        for index, method in enumerate(service["methods"]):
            output.append("%s Client::%s(const %s& %s)" % (method["output_type"], method["name"], method["input_type"], "request"))
            output.append("{")
            output.append("    %s response;" % (method["output_type"]))
            output.append("    call(%s, request, response);" % (index))
            output.append("    return response;")
            output.append("}")
            output.append("")

        # Generate server
        output.append("Server::Server(uint16_t port) :")
        output.append("    Rpc::Server(port)")
        output.append("{")
        output.append("}")
        output.append("")

        output.append("Server::~Server()")
        output.append("{")
        output.append("}")
        output.append("")

        output.append("::google::protobuf::Message* Server::get_request_prototype(uint8_t method)")
        output.append("{")
        output.append("    ::google::protobuf::Message* request_prototype;")
        output.append("    switch (method)")
        output.append("    {")
        for index, method in enumerate(service["methods"]):
            output.append("    case %d:" % (index))
            output.append("        request_prototype = new %s();" % (method["input_type"]))
            output.append("        break;")
        output.append("    default:")
        output.append("        throw(std::runtime_error(\"Unknown method \" + std::to_string(method)));")
        output.append("        break;")
        output.append("    }")
        output.append("")
        output.append("    return request_prototype;")
        output.append("}")
        output.append("")

        output.append("::google::protobuf::Message* Server::get_response_prototype(uint8_t method)")
        output.append("{")
        output.append("    ::google::protobuf::Message* response_prototype;")
        output.append("    switch (method)")
        output.append("    {")
        for index, method in enumerate(service["methods"]):
            output.append("    case %d:" % (index))
            output.append("        response_prototype = new %s();" % (method["output_type"]))
            output.append("        break;")
        output.append("    default:")
        output.append("        throw(std::runtime_error(\"Unknown method \" + std::to_string(method)));")
        output.append("        break;")
        output.append("    }")
        output.append("")
        output.append("    return response_prototype;")
        output.append("}")
        output.append("")

        output.append("void Server::call(uint8_t method, const ::google::protobuf::Message& request, ::google::protobuf::Message& response)")
        output.append("{")
        output.append("    switch (method)")
        output.append("    {")
        for index, method in enumerate(service["methods"]):
            output.append("    case %d:" % (index))
            output.append("        %s(::google::protobuf::down_cast<const %s&>(request), ::google::protobuf::down_cast<%s&>(response));" % (method["name"], method["input_type"], method["output_type"]))
            output.append("        break;") 
        output.append("    default:")
        output.append("        throw(std::runtime_error(\"Unknown method \" + std::to_string(method)));")
        output.append("        break;")
        output.append("    }")
        output.append("}")
        output.append("")

    output.append("}")

    return "\n".join(output)

if __name__ == '__main__':
    # Read request message from stdin
    data = sys.stdin.buffer.read()

    # Parse request
    request = plugin.CodeGeneratorRequest()
    request.ParseFromString(data)

    # Create response
    response = plugin.CodeGeneratorResponse()

    # Generate code
    generate_code(request, response)

    # Serialise response message
    output = response.SerializeToString()

    # Write to stdout
    sys.stdout.buffer.write(output)
