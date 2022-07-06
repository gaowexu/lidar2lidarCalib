import copy
import warnings
import numpy as np
import lzf
import struct
from sensor_msgs.msg import PointField


pc2_pcd_type_mappings = [(PointField.INT8, ('I', 1)),
                         (PointField.UINT8, ('U', 1)),
                         (PointField.INT16, ('I', 2)),
                         (PointField.UINT16, ('U', 2)),
                         (PointField.INT32, ('I', 4)),
                         (PointField.UINT32, ('U', 4)),
                         (PointField.FLOAT32, ('F', 4)),
                         (PointField.FLOAT64, ('F', 8))]
pc2_type_to_pcd_type = dict(pc2_pcd_type_mappings)

# prefix to the names of dummy fields we add to get byte alignment correct. this needs to not
# clash with any actual field names
DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]

pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

# sizes (in bytes) of PointField types
pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}


def _metadata_is_consistent(metadata):
    """ Sanity check for metadata. Just some basic checks.
    """
    checks = []
    required = ('version', 'fields', 'size', 'width', 'height', 'points',
                'viewpoint', 'data')
    for f in required:
        if f not in metadata:
            print('%s required' % f)
    checks.append((lambda m: all([k in m for k in required]),
                   'missing field'))
    checks.append((lambda m: len(m['type']) == len(m['count']) ==
                   len(m['fields']),
                   'length of type, count and fields must be equal'))
    checks.append((lambda m: m['height'] > 0,
                   'height must be greater than 0'))
    checks.append((lambda m: m['width'] > 0,
                   'width must be greater than 0'))
    checks.append((lambda m: m['points'] > 0,
                   'points must be greater than 0'))
    checks.append((lambda m: m['data'].lower() in ('ascii', 'binary',
                   'binary_compressed'),
                   'unknown data type:'
                   'should be ascii/binary/binary_compressed'))
    ok = True
    for check, msg in checks:
        if not check(metadata):
            print('error:', msg)
            ok = False
    return ok


def write_header(metadata, rename_padding=False):
    """ Given metadata as dictionary, return a string header.
    """
    template = """\
VERSION {version}
FIELDS {fields}
SIZE {size}
TYPE {type}
COUNT {count}
WIDTH {width}
HEIGHT {height}
VIEWPOINT {viewpoint}
POINTS {points}
DATA {data}
"""
    str_metadata = metadata.copy()

    if not rename_padding:
        str_metadata['fields'] = ' '.join(metadata['fields'])
    else:
        new_fields = []
        for f in metadata['fields']:
            if f == '_':
                new_fields.append('padding')
            else:
                new_fields.append(f)
        str_metadata['fields'] = ' '.join(new_fields)
    str_metadata['size'] = ' '.join(map(str, metadata['size']))
    str_metadata['type'] = ' '.join(metadata['type'])
    str_metadata['count'] = ' '.join(map(str, metadata['count']))
    str_metadata['width'] = str(metadata['width'])
    str_metadata['height'] = str(metadata['height'])
    str_metadata['viewpoint'] = ' '.join(map(str, metadata['viewpoint']))
    str_metadata['points'] = str(metadata['points'])
    tmpl = template.format(**str_metadata)
    return tmpl


def build_ascii_fmtstr(pc):
    """ Make a format string for printing to ascii.

    Note %.8f is minimum for rgb.
    """
    fmtstr = []
    for t, cnt in zip(pc.type, pc.count):
        if t == 'F':
            fmtstr.extend(['%.10f']*cnt)
        elif t == 'I':
            fmtstr.extend(['%d']*cnt)
        elif t == 'U':
            fmtstr.extend(['%u']*cnt)
        else:
            raise ValueError("don't know about type %s" % t)
    return fmtstr


def point_cloud_to_fileobj(pc, fileobj, data_compression=None):
    """ Write pointcloud as .pcd to fileobj.
    If data_compression is not None it overrides pc.data.
    """
    metadata = pc.get_metadata()
    if data_compression is not None:
        data_compression = data_compression.lower()
        assert(data_compression in ('ascii', 'binary', 'binary_compressed'))
        metadata['data'] = data_compression

    header = write_header(metadata)
    fileobj.write(header)
    if metadata['data'].lower() == 'ascii':
        fmtstr = build_ascii_fmtstr(pc)
        np.savetxt(fileobj, pc.pc_data, fmt=fmtstr)
    elif metadata['data'].lower() == 'binary':
        fileobj.write(pc.pc_data.tostring('C'))
    elif metadata['data'].lower() == 'binary_compressed':
        # TODO
        # a '_' field is ignored by pcl and breakes compressed point clouds.
        # changing '_' to '_padding' or other name fixes this.
        # admittedly padding shouldn't be compressed in the first place.
        # reorder to column-by-column
        uncompressed_lst = []
        for fieldname in pc.pc_data.dtype.names:
            column = np.ascontiguousarray(pc.pc_data[fieldname]).tostring('C')
            uncompressed_lst.append(column)

        uncompressed = b''.join(uncompressed_lst)
        uncompressed_size = len(uncompressed)
        buf = lzf.compress(uncompressed)
        if buf is None:
            # compression didn't shrink the file
            # TODO what do to do in this case when reading?
            buf = uncompressed
            compressed_size = uncompressed_size
        else:
            compressed_size = len(buf)
        fmt = 'II'
        content = struct.pack(fmt, compressed_size, uncompressed_size)
        content_dump = "".join(map(chr, content))
        fileobj.write(content_dump)
        fileobj.write("".join(map(chr, buf)))
    else:
        raise ValueError('unknown DATA type')
    # we can't close because if it's stringio buf then we can't get value after


class PointCloud(object):
    """ Wrapper for point cloud data.

    The variable members of this class parallel the ones used by
    the PCD metadata (and similar to PCL and ROS PointCloud2 messages),

    ``pc_data`` holds the actual data as a structured numpy array.

    The other relevant metadata variables are:

    - ``version``: Version, usually .7
    - ``fields``: Field names, e.g. ``['x', 'y' 'z']``.
    - ``size.`: Field sizes in bytes, e.g. ``[4, 4, 4]``.
    - ``count``: Counts per field e.g. ``[1, 1, 1]``. NB: Multi-count field
      support is sketchy.
    - ``width``: Number of points, for unstructured point clouds (assumed by
      most operations).
    - ``height``: 1 for unstructured point clouds (again, what we assume most
      of the time.
    - ``viewpoint``: A pose for the viewpoint of the cloud, as
      x y z qw qx qy qz, e.g. ``[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]``.
    - ``points``: Number of points.
    - ``type``: Data type of each field, e.g. ``[F, F, F]``.
    - ``data``: Data storage format. One of ``ascii``, ``binary`` or ``binary_compressed``.

    See `PCL docs <http://pointclouds.org/documentation/tutorials/pcd_file_format.php>`__
    for more information.
    """

    def __init__(self, metadata, pc_data):
        self.metadata_keys = metadata.keys()
        self.__dict__.update(metadata)
        self.pc_data = pc_data

    def get_metadata(self):
        """ returns copy of metadata """
        metadata = {}
        for k in self.metadata_keys:
            metadata[k] = copy.copy(getattr(self, k))
        return metadata

    def save_pcd(self, fname, compression=None):
        with open(fname, 'w') as f:
            point_cloud_to_fileobj(self, f, compression)

    @staticmethod
    def pointcloud2_to_dtype(cloud_msg):
        '''Convert a list of PointFields to a numpy record datatype.
        '''
        offset = 0
        np_dtype_list = []
        for f in cloud_msg.fields:
            while offset < f.offset:
                # might be extra padding between fields
                np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
                offset += 1
            np_dtype_list.append((f.name, pftype_to_nptype[f.datatype]))
            offset += pftype_sizes[f.datatype]

        # might be extra padding between points
        while offset < cloud_msg.point_step:
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        return np_dtype_list

    @staticmethod
    def pointcloud2_to_dtype(cloud_msg):
        '''Convert a list of PointFields to a numpy record datatype.
        '''
        offset = 0
        np_dtype_list = []
        for f in cloud_msg.fields:
            while offset < f.offset:
                # might be extra padding between fields
                np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
                offset += 1
            np_dtype_list.append((f.name, pftype_to_nptype[f.datatype]))
            offset += pftype_sizes[f.datatype]

        # might be extra padding between points
        while offset < cloud_msg.point_step:
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        return np_dtype_list

    @staticmethod
    def pointcloud2_to_array(cloud_msg, remove_padding=True):
        ''' Converts a rospy PointCloud2 message to a numpy recordarray

        Reshapes the returned array to have shape (height, width), even if the height is 1.

        The reason for using np.fromstring rather than struct.unpack is speed... especially
        for large point clouds, this will be <much> faster.
        '''
        # construct a numpy record type equivalent to the point type of this cloud
        dtype_list = PointCloud.pointcloud2_to_dtype(cloud_msg)

        # parse the cloud into an array
        cloud_arr = np.fromstring(cloud_msg.data.tobytes(), dtype_list)

        # remove the dummy fields that were added
        if remove_padding:
            cloud_arr = cloud_arr[
                [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

    @staticmethod
    def from_msg(msg, squeeze=True):
        """ from pointcloud2 msg
        squeeze: fix when clouds get 1 as first dim
        """
        md = {'version': .7,
              'fields': [],
              'size': [],
              'count': [],
              'width': msg.width,
              'height': msg.height,
              'viewpoint': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
              'points': 0,
              'type': [],
              'data': 'binary_compressed'}
        for field in msg.fields:
            md['fields'].append(field.name)
            t, s = pc2_type_to_pcd_type[field.datatype]
            md['type'].append(t)
            md['size'].append(s)
            # TODO handle multicount correctly
            if field.count > 1:
                warnings.warn('fields with count > 1 are not well tested')
            md['count'].append(field.count)
        pc_array = PointCloud.pointcloud2_to_array(msg)

        pc_data = pc_array.reshape(-1)
        md['height'], md['width'] = pc_array.shape
        md['points'] = len(pc_data)
        pc = PointCloud(md, pc_data)
        return pc
