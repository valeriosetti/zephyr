/*
 * Copyright (c) 2026 BayLibre SAS
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SECURE_STORAGE_UID_H
#define SECURE_STORAGE_UID_H

#include <stdint.h>
#include <zephyr/secure_storage/common.h>
#include <psa/storage_common.h>

#ifdef CONFIG_SECURE_STORAGE_64_BIT_UID

/** The UID (caller + entry IDs) of an ITS entry. */
typedef struct __packed {
	psa_storage_uid_t uid;
	secure_storage_caller_id_t caller_id;
} secure_storage_uid_t;

#else

#define SECURE_STORAGE_UID_BIT_SIZE 30
#define SECURE_STORAGE_CALLER_ID_BIT_SIZE 2

/** @brief The UID (caller + entry IDs) of an ITS entry.
 * This is a packed, 32-bit version of `psa_storage_uid_t` which allows storing
 * smaller IDs compared to the 64-bit ones that PSA Secure Storage specifies.
 * Zephyr defines ranges of IDs to be used by different users of the API (subsystems, application)
 * which guarantees 1. no collisions and 2. that the IDs used fit within `uid`.
 * @see @ref zephyr/psa/key_ids.h and the other header files under `zephyr/psa`.
 */
typedef struct {
	psa_storage_uid_t uid : SECURE_STORAGE_UID_BIT_SIZE;
	secure_storage_caller_id_t caller_id : SECURE_STORAGE_CALLER_ID_BIT_SIZE;
} secure_storage_uid_t;

#endif /* CONFIG_SECURE_STORAGE_64_BIT_UID */

/* For logging a `secure_storage_uid_t`, whose width depends on the configuration. */
#ifdef CONFIG_SECURE_STORAGE_64_BIT_UID
#define UID_FMT           "%u/%#llx"
#define UID_ARGS(its_uid) (its_uid).caller_id, (unsigned long long)(its_uid).uid
#else
#define UID_FMT           "%u/%#lx"
#define UID_ARGS(its_uid) (its_uid).caller_id, (unsigned long)(its_uid).uid
#endif

psa_status_t secure_storage_make_uid(secure_storage_caller_id_t caller_id,
				     psa_storage_uid_t uid,
				     secure_storage_uid_t *out_uid);

#endif /* SECURE_STORAGE_UID_H */
