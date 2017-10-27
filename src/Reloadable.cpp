#include <ras_group8_util/Reloadable.hpp>

// STD
#include <string>

namespace ras_group8_util {

template<class N>
Reloadable<N>::Reloadable(ros::NodeHandle& node_handle,
                          N (*setup)(ros::NodeHandle&))
    : node_handle_(node_handle),
      setup_(setup),
      node_(setup(node_handle_))
{
  /* Setup a reload service */
  reload_service_ =
    node_handle_.advertiseService("reload", &Reloadable<N>::reload, this);
}

template<class N>
Reloadable<N>::~Reloadable()
{
}

template<class N>
N& Reloadable<N>::node()
{
  return node_;
}

template<class N>
void Reloadable<N>::reload()
{
  /* Recreate the node using the setup function */
  node_ = setup_(node_handle_);
}


} /* namespace */