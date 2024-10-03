YALMIP
======

`YALMIP <https://yalmip.github.io/>`_ supports the OSQP solver. You can easily define problems in high-level format and then specify OSQP by simply setting

.. code:: matlab

   options = sdpsettings('solver', 'osqp', 'osqp.max_iter', 2000);


where we set the :code:`max_iter` option to :code:`2000`.
