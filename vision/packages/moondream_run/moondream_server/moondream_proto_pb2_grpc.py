# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc
import warnings

import moondream_proto_pb2 as moondream__proto__pb2

GRPC_GENERATED_VERSION = '1.71.0'
GRPC_VERSION = grpc.__version__
_version_not_supported = False

try:
    from grpc._utilities import first_version_is_lower
    _version_not_supported = first_version_is_lower(GRPC_VERSION, GRPC_GENERATED_VERSION)
except ImportError:
    _version_not_supported = True

if _version_not_supported:
    raise RuntimeError(
        f'The grpc package installed is at version {GRPC_VERSION},'
        + f' but the generated code in moondream_proto_pb2_grpc.py depends on'
        + f' grpcio>={GRPC_GENERATED_VERSION}.'
        + f' Please upgrade your grpc module to grpcio>={GRPC_GENERATED_VERSION}'
        + f' or downgrade your generated code using grpcio-tools<={GRPC_VERSION}.'
    )


class MoonDreamServiceStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.EncodeImage = channel.unary_unary(
                '/moondream.MoonDreamService/EncodeImage',
                request_serializer=moondream__proto__pb2.ImageRequest.SerializeToString,
                response_deserializer=moondream__proto__pb2.EncodedImageResponse.FromString,
                _registered_method=True)
        self.FindBeverage = channel.unary_unary(
                '/moondream.MoonDreamService/FindBeverage',
                request_serializer=moondream__proto__pb2.FindBeverageRequest.SerializeToString,
                response_deserializer=moondream__proto__pb2.BeveragePositionResponse.FromString,
                _registered_method=True)
        self.Query = channel.unary_unary(
                '/moondream.MoonDreamService/Query',
                request_serializer=moondream__proto__pb2.QueryRequest.SerializeToString,
                response_deserializer=moondream__proto__pb2.QueryResponse.FromString,
                _registered_method=True)


class MoonDreamServiceServicer(object):
    """Missing associated documentation comment in .proto file."""

    def EncodeImage(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def FindBeverage(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Query(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_MoonDreamServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'EncodeImage': grpc.unary_unary_rpc_method_handler(
                    servicer.EncodeImage,
                    request_deserializer=moondream__proto__pb2.ImageRequest.FromString,
                    response_serializer=moondream__proto__pb2.EncodedImageResponse.SerializeToString,
            ),
            'FindBeverage': grpc.unary_unary_rpc_method_handler(
                    servicer.FindBeverage,
                    request_deserializer=moondream__proto__pb2.FindBeverageRequest.FromString,
                    response_serializer=moondream__proto__pb2.BeveragePositionResponse.SerializeToString,
            ),
            'Query': grpc.unary_unary_rpc_method_handler(
                    servicer.Query,
                    request_deserializer=moondream__proto__pb2.QueryRequest.FromString,
                    response_serializer=moondream__proto__pb2.QueryResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'moondream.MoonDreamService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
    server.add_registered_method_handlers('moondream.MoonDreamService', rpc_method_handlers)


 # This class is part of an EXPERIMENTAL API.
class MoonDreamService(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def EncodeImage(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/moondream.MoonDreamService/EncodeImage',
            moondream__proto__pb2.ImageRequest.SerializeToString,
            moondream__proto__pb2.EncodedImageResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def FindBeverage(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/moondream.MoonDreamService/FindBeverage',
            moondream__proto__pb2.FindBeverageRequest.SerializeToString,
            moondream__proto__pb2.BeveragePositionResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def Query(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/moondream.MoonDreamService/Query',
            moondream__proto__pb2.QueryRequest.SerializeToString,
            moondream__proto__pb2.QueryResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)
