from networks.default_network import Network as DefaultNetwork
from networks.batch_norm_network import Network as BatchNormNetwork
from networks.sigmoid_network import Network as SigmoidNetwork
from networks.large_default import Network as LargeDefaultNetwork
from networks.large_bn import Network as LargeBatchNormNetwork
from networks.long_default import Network as LongDefaultNetwork
from networks.very_long_default import Network as VeryLongDefaultNetwork
from networks.wide_ff import Network as WideFFNetwork
from networks.small_ff import Network as SmallFFNetwork

 
def return_network(network_name):
    if network_name == "default":
        return DefaultNetwork
    if network_name == "batch_norm":
        return BatchNormNetwork
    if network_name == "sigmoid":
        return SigmoidNetwork
    if network_name == "large_default":
        return LargeDefaultNetwork
    if network_name == "large_batch_norm":
        return LargeBatchNormNetwork
    if network_name == "long_default":
        return LongDefaultNetwork
    if network_name == "very_long_default":
        return VeryLongDefaultNetwork
    if network_name == "wide_ff":
        return WideFFNetwork
    if network_name == "small_ff":
        return SmallFFNetwork
