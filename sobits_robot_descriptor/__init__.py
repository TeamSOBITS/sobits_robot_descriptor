# Copyright (c) 2026, Team SOBITS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Shared robot descriptor (schema v2) loader for SOBITS tools."""

from .loader import ENV_PATH, load, load_file, parse, read_yaml, resolve_path
from .schema import (
    BASE_INTERFACES, INTERFACES, KINDS, SCHEMA_VERSION, BaseControllerSpec, CameraSpec,
    DescriptorError, EEControlSpec, EESpec, GroupSpec, ImuSpec, LidarSpec,
    MobileBaseSpec, RobotDescriptor, StreamSpec, UrdfSpec, derived_topics,
)
from .validate import check, validate

__all__ = [
    'BASE_INTERFACES', 'ENV_PATH', 'INTERFACES', 'KINDS', 'SCHEMA_VERSION',
    'BaseControllerSpec', 'CameraSpec', 'DescriptorError', 'EEControlSpec',
    'EESpec', 'GroupSpec', 'ImuSpec', 'LidarSpec', 'MobileBaseSpec',
    'RobotDescriptor', 'StreamSpec', 'UrdfSpec',
    'check', 'derived_topics', 'load', 'load_file', 'parse', 'read_yaml',
    'resolve_path', 'validate',
]
