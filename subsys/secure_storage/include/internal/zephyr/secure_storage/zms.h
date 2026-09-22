/*
 * Copyright (c) 2026 BayLibre SAS
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SECURE_STORAGE_ZMS_H
#define SECURE_STORAGE_ZMS_H

#include <stdint.h>
#include <zephyr/kvss/zms.h>
#include <zephyr/secure_storage/uid.h>
#include <psa/error.h>

psa_status_t secure_storage_store_set(struct zms_fs *zms, secure_storage_uid_t uid,
				      size_t data_length, const void *data);

psa_status_t secure_storage_store_get(struct zms_fs *zms, secure_storage_uid_t uid,
				      size_t data_size, void *data, size_t *data_length);

psa_status_t secure_storage_store_remove(struct zms_fs *zms, secure_storage_uid_t uid);

#endif /* SECURE_STORAGE_ZMS_H */
